#include <chrono>
#include <ctime>
#include <memory>
#include <functional>
#include <string>

// Uldaq
#include "uldaq.h"
#include "utility.h"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "uldaq_msgs/msg/buffer.hpp"
#include "uldaq_msgs/msg/measurement.hpp"

// Node
#include "buffer_pub.hpp"

using namespace std;
namespace uldaq_ros {
UldaqPublisher::UldaqPublisher(const rclcpp::NodeOptions& options)
  : Node("uldaq_publisher", options), samples_read(0) {
  declare_parameter<int>("v_range", 5);
  declare_parameter<int>("chan_num", 8);
  declare_parameter<int>("rate", 100);
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

  int num_chan = parameters_client->get_parameter<int>("chan_num"); 
  int volt_range = parameters_client->get_parameter<int>("v_range");
  int daq_rate = parameters_client->get_parameter<int>("rate");

  bufpub = create_publisher<uldaq_msgs::msg::Buffer>("uldaq_buffer", 10);
  recpub = create_publisher<uldaq_msgs::msg::Measurement>("uldaq_measurement", 10);
  past_scan = 0;
  samples_read = 0;

  DaqDeviceDescriptor devDescriptors[MAX_DEV_COUNT];
  DaqDeviceInterface interfaceType = ANY_IFC;
  DaqDeviceDescriptor DeviceDescriptor;
  unsigned int numDevs = MAX_DEV_COUNT;
  UlError detectError = ERR_NO_ERROR;

  // Acquire device(s)
  detectError = ulGetDaqDeviceInventory(interfaceType, devDescriptors, &numDevs);
  if(detectError != 0){
    RCLCPP_ERROR(get_logger(), "Cannot acquire device inventory\n");
    rclcpp::shutdown();
  }
  // verify at least one DAQ device is detected
  if (numDevs == 0) {
    RCLCPP_ERROR(get_logger(), "No DAQ device is detected\n");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(get_logger(),"Found %d DAQ device(s)\n", numDevs);
	for (int i = 0; i < (int) numDevs; i++) {
		RCLCPP_INFO(get_logger(),"  [%d] %s: (%s)\n", i, devDescriptors[i].productName, devDescriptors[i].uniqueId);
  }
  DeviceDescriptor = devDescriptors[0];
  // get a handle to the DAQ device associated with the first descriptor
  deviceHandle = ulCreateDaqDevice(DeviceDescriptor);
  detectError = ulConnectDaqDevice(deviceHandle);

  // setup DAQ
  short LowChan = 0;
  short HighChan = num_chan - 1;
  double rated = (double)daq_rate;
  long samplesPerChan = daq_rate*10; // holds 10s of data
  long numBufferPoints = num_chan * samplesPerChan;
  std::cout << "rate" << daq_rate << "chans" << num_chan << "samples" << samplesPerChan << "points" << numBufferPoints << endl;
  double* buffer = (double*) malloc(numBufferPoints * sizeof(double));
  if(buffer == 0){
    RCLCPP_ERROR(get_logger(), "Out of memory\n" );
    rclcpp::shutdown();
  }
  Range gain = UldaqPublisher::getGain(volt_range);
  // DAQ is master to Vectornav, attached VN_SYNCIN to CLKOUT on DAQ.
  ScanOption scan_options = (ScanOption) (SO_DEFAULTIO | SO_CONTINUOUS ); //| SO_PACEROUT);
  AInScanFlag flags = AINSCAN_FF_DEFAULT;

  // setup scan event for the DAQ
  long event_on_samples = samplesPerChan/20; // trigger event every 0.5 seconds.
  scan_event = (DE_ON_DATA_AVAILABLE);
  ScanEventParameters user_data;
  user_data.buffer = buffer;
  user_data.buffer_size = numBufferPoints; 
  user_data.lowChan = LowChan;
  user_data.highChan = HighChan;
  user_data.node = this;
  // TODO: if this bind ends up failing, check how the ros2/vectornav node does it
  detectError = ulEnableEvent(deviceHandle, scan_event, event_on_samples, &UldaqPublisher::daqEventHandle, &user_data); // if this complains about static we gonna have to get a little weird with the publishers
  if (detectError != 0){
    RCLCPP_ERROR(get_logger(), "Could not enable event\n");
    rclcpp::shutdown();
  }

  usleep(20000); // increase stability of the deviceHandle when on the external clock

  detectError = ulAInScan(deviceHandle, LowChan, HighChan, AI_SINGLE_ENDED, gain, samplesPerChan, &rated, scan_options,  flags, buffer);
  if (detectError != 0){
    RCLCPP_ERROR(get_logger(), "Couldn't start scan\n");
    rclcpp::shutdown();
  }
  RCLCPP_INFO(get_logger(), "successfully initialized!");
}

UldaqPublisher::~UldaqPublisher(){
  // wrap up daq
  ulAInScanStop(deviceHandle);
  ulDisableEvent(deviceHandle, scan_event);
  ulDisconnectDaqDevice(deviceHandle);
}
Range UldaqPublisher::getGain(int vRange) {
  Range gain;
  switch (vRange) {
    case(1): {
         gain = BIP1VOLTS;
         return gain;
         break;
       }
    case(2): {
         gain = BIP2VOLTS;
         return gain;
         break;
       }
    case(5): {
         gain = BIP5VOLTS;
         return gain;
         break;
       }
    case(10): {
          gain = BIP10VOLTS;
          return gain;
          break;
        }
    default: {
         gain = BIP5VOLTS;
         return gain;
         break;
       }
  }
}
void UldaqPublisher::daqEventHandle(DaqDeviceHandle daqDeviceHandle, DaqEventType eventType, unsigned long long eventData, void* userData)
{ 
  ScanEventParameters* scanEventParameters = (ScanEventParameters*)userData;
  scanEventParameters->node->_daqEventHandle(daqDeviceHandle, eventType, eventData, userData);
}
void UldaqPublisher::_daqEventHandle(DaqDeviceHandle daqDeviceHandle, DaqEventType eventType, unsigned long long eventData, void* userData)
{
  // ROS2 Messages, msg.data is a std::vector
  auto recent_measurement = uldaq_msgs::msg::Measurement();
  auto full_buffer = uldaq_msgs::msg::Buffer();


  // handle time
  const auto p0 = std::chrono::time_point<std::chrono::high_resolution_clock>{};
  const auto p3 = std::chrono::high_resolution_clock::now();

  auto tstamp = p3 - p0;
  int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp).count();
  int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp).count() % 1000000000UL;
  rclcpp::Time n(sec, nsec);
  recent_measurement.header.stamp = n;
  full_buffer.header.stamp = n;

  DaqDeviceDescriptor activeDevDescriptor;
  ulGetDaqDeviceDescriptor(daqDeviceHandle, &activeDevDescriptor);
  UlError err = ERR_NO_ERROR;

  ScanEventParameters* scanEventParameters = (ScanEventParameters*) userData;

  int chan_count = scanEventParameters->highChan - scanEventParameters->lowChan + 1; 
  unsigned long long total_samples = eventData*chan_count; 
  // long number_of_samples; 
  double *current_doubles = (double *)malloc(chan_count*sizeof(double)); // most recent reading

  if (eventType == DE_ON_DATA_AVAILABLE) {
    unsigned long sample_index = total_samples % scanEventParameters->buffer_size;
    // essentially, using std::copy and std::memcpy we are trying to replicate this behavior:
    // 1. take the byte buffer and find the start and end index for our current samples
    // 2. using the start and end index copy the current samples into a std::vector<byte> full_buffer
    // 3. take the last current sample and store it into std::vector<double> recent_measurement
    // std::copy(starting_memaddress, ending_memaddress, destination_iterator);
    // std:memcpy(dest_pointer, src_pointer, count)
    //
    if (sample_index < past_scan) { // buffer wrap around
	    std::copy(&(scanEventParameters->buffer[past_scan]), &(scanEventParameters->buffer[scanEventParameters->buffer_size - 1]), 
			    std::back_inserter(full_buffer.data));
	    std::copy(&(scanEventParameters->buffer[0]), &(scanEventParameters->buffer[sample_index]), 
			    std::back_inserter(full_buffer.data));
    }
    else { 
	    std::copy(&(scanEventParameters->buffer[past_scan]), &(scanEventParameters->buffer[sample_index]), 
			    std::back_inserter(full_buffer.data));
    }
    std::copy(&(scanEventParameters->buffer[sample_index - chan_count]), &(scanEventParameters->buffer[sample_index]), current_doubles);
    for(int i = 0; i < chan_count; i++) {
      recent_measurement.data[i] = current_doubles[i];
    }
    past_scan = sample_index;
  } else if (eventType == DE_ON_INPUT_SCAN_ERROR) {
    cout << "error" << endl;
    err = (UlError) eventData;
    char errMsg[ERR_MSG_LEN];
    ulGetErrMsg(err, errMsg);
    RCLCPP_ERROR(get_logger(), "Error Code: %d \n", err);
    RCLCPP_ERROR(get_logger(), "Error Message: %s \n", errMsg);
  } else if (eventType == DE_ON_END_OF_INPUT_SCAN) {
    cout << "scan complete" << endl;
    RCLCPP_ERROR(get_logger(), "\nThe scan using device %s (%s) is complete \n", activeDevDescriptor.productName, activeDevDescriptor.uniqueId);
  }
  free(current_doubles);
  bufpub->publish(full_buffer);
  recpub->publish(recent_measurement);
}
} // namespace uldaq_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uldaq_ros::UldaqPublisher)
