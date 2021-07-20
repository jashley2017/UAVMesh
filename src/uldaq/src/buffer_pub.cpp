#include <chrono>
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

using namespace std;

class UldaqPublisher : public rclcpp::Node
{
  public:
    UldaqPublisher()
    : Node("uldaq_publisher")
    {
      this->declare_parameter<int>("v_range", 5);
      this->declare_parameter<int>("chan_num", 8);
      this->declare_parameter<int>("rate", 1000);

      int num_chan;
      int volt_range;
      int daq_rate;
      this->get_parameter("chan_num", num_chan); // wtf why arent these the same? I am an asshole jeez...
      this->get_parameter("v_range", volt_range);
      this->get_parameter("rate", daq_rate);

      bufpub = this->create_publisher<uldaq_msgs::msg::Buffer>("uldaq_buffer", 10);
      recpub = this->create_publisher<uldaq_msgs::msg::Measurement>("uldaq_measurement", 10);

      DaqDeviceDescriptor devDescriptors[MAX_DEV_COUNT];
      DaqDeviceInterface interfaceType = ANY_IFC;
      DaqDeviceDescriptor DeviceDescriptor;
      DaqDeviceHandle deviceHandle;
      unsigned int numDevs = MAX_DEV_COUNT;
      UlError detectError = ERR_NO_ERROR;

      // Acquire device(s)
      detectError = ulGetDaqDeviceInventory(interfaceType, devDescriptors, &numDevs);
      if(handleError(detectError, "Cannot acquire device inventory\n")){
      }
      // verify at least one DAQ device is detected
      if (numDevs == 0) {
        cerr << "No DAQ device is detected\n" << endl;
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
      double* buffer = (double*) malloc(numBufferPoints * sizeof(double));
      if(buffer == 0){
        cout << "Out of memory\n" << endl;
      }
      Range gain = UldaqPublisher::getGain(volt_range);
      // DAQ is master to Vectornav, attached VN_SYNCIN to CLKOUT on DAQ.
      ScanOption options = (ScanOption) (SO_DEFAULTIO | SO_CONTINUOUS | SO_PACEROUT);
      AInScanFlag flags = AINSCAN_FF_DEFAULT;

      // setup scan event for the DAQ
      long event_on_samples = samplesPerChan/100; // trigger event every 0.1 seconds.
      DaqEventType scan_event = (DE_ON_DATA_AVAILABLE);
      ScanEventParameters user_data;
      user_data.buffer = buffer;
      user_data.buffer_size = numBufferPoints; 
      user_data.lowChan = LowChan;
      user_data.highChan = HighChan;
      detectError = ulEnableEvent(deviceHandle, scan_event, event_on_samples, std::bind(&UldaqPublisher::daqEventHandle, this, deviceHandle, scan_event, event_on_samples, user_data), &user_data); // if this complains about static we gonna have to get a little weird with the publishers
      if (handleError(detectError, "Could not enable event\n")){
      }

      usleep(20000); // increase stability of the deviceHandle when on the external clock

      detectError = ulAInScan(deviceHandle, LowChan, HighChan, AI_SINGLE_ENDED, gain, samplesPerChan, &rated, options,  flags, buffer);
      if (handleError(detectError, "Couldn't start scan\n")){
      }

      UldaqPublisher::awaitDaqEvents(); // TODO: expose the Daq pthread event to be able to spin on
      // wrap up daq
      ulAInScanStop(deviceHandle);
      ulDisableEvent(deviceHandle, scan_event);
      ulDisconnectDaqDevice(deviceHandle);
    }
  private:
    Range getGain(int vRange) {
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
    void awaitDaqEvents()
    {
      while(rclcpp::ok()) { 
        // this should probably yield to the daq thread once we get it.
      }
    }
    void daqEventHandle(DaqDeviceHandle daqDeviceHandle, DaqEventType eventType, unsigned long long eventData, void* userData)
    { 
      // ROS2 Messages, msg.data is a std::vector
      auto recent_measurement = uldaq_msgs::msg::Measurement();
      auto full_buffer = uldaq_msgs::msg::Buffer();

      DaqDeviceDescriptor activeDevDescriptor;
      ulGetDaqDeviceDescriptor(daqDeviceHandle, &activeDevDescriptor);
      UlError err = ERR_NO_ERROR;

      ScanEventParameters* scanEventParameters = (ScanEventParameters*) userData;
      int chan_count = scanEventParameters->highChan - scanEventParameters->lowChan + 1; 
      unsigned long long total_samples = eventData*chan_count; 
      long number_of_samples; 
      double current_doubles[chan_count]; // most recent reading

      if (eventType == DE_ON_DATA_AVAILABLE) {
        unsigned long sample_index = total_samples % scanEventParameters->buffer_size;
        // TODO: the following copy lines are a lot to unpack and may not be correct
        // essentially, using std::copy and std::memcpy we are trying to replicate this behavior:
        // 1. take the byte buffer and find the start and end index for our current samples
        // 2. using the start and end index copy the current samples into a std::vector<byte> full_buffer
        // 3. take the last current sample and store it into std::vector<double> recent_measurement
        // std::copy(starting_memaddress, ending_memaddress, destination_iterator);
        // std:memcpy(dest_pointer, src_pointer, count)
        if (sample_index < past_scan) { // buffer wrap around
          // copy the current valid buffer range to our message
          number_of_samples = scanEventParameters->buffer_size - past_scan; // go to the end of the buffer
          std::copy(&(scanEventParameters->buffer[past_scan]), &(scanEventParameters->buffer[past_scan+sizeof(double)*(number_of_samples-1)]), full_buffer.data.begin()) 
          number_of_samples = sample_index;
          std::copy(&(scanEventParameters->buffer[0]), &(scanEventParameters->buffer[past_scan+sizeof(double)*(number_of_samples-1)]), back_inserter(full_buffer.data))  // back_inserter is an iterator of push_back
          // convert the most recent reading to its double form
          std::memcpy(current_doubles, &(scanEventParameters->buffer[past_scan+sizeof(double)*(number_of_samples-1) - sizeof(double)*chan_count]), sizeof(double)*chan_count)
          std::copy(current_doubles, current_doubles+chan_count-1, recent_measurement.data.begin())
        } else { // normal operation
          number_of_samples = sample_index - past_scan;
          // copy the current valid buffer range to our message
          std::copy(&(scanEventParameters->buffer[past_scan]), &(scanEventParameters->buffer[past_scan+sizeof(double)*(number_of_samples-1)]), full_buffer.data.begin()) 
          // convert the most recent reading to its double form
          std::memcpy(current_doubles, &(scanEventParameters->buffer[past_scan+sizeof(double)*(number_of_samples-1) - sizeof(double)*chan_count]), sizeof(double)*chan_count)
          std::copy(current_doubles, current_doubles+chan_count-1, recent_measurement.data.begin())
        }
        past_scan = sample_index;
      } else if (eventType == DE_ON_INPUT_SCAN_ERROR) {
        err = (UlError) eventData;
        char errMsg[ERR_MSG_LEN];
        ulGetErrMsg(err, errMsg);
        printf("Error Code: %d \n", err);
        printf("Error Message: %s \n", errMsg);
      } else if (eventType == DE_ON_END_OF_INPUT_SCAN) {
        printf("\nThe scan using device %s (%s) is complete \n", activeDevDescriptor.productName, activeDevDescriptor.uniqueId);
      }
      bufpub->publish(full_buffer);
      recpub->publish(recent_measurement);
    }
    unsigned long past_scan;
    rclcpp::Publisher<uldaq_msgs::msg::Buffer>::SharedPtr bufpub;
    rclcpp::Publisher<uldaq_msgs::msg::Measurement>::SharedPtr recpub;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv); 
  rclcpp::spin(std::make_shared<UldaqPublisher>());
  rclcpp::shutdown();
  return 0;
}
