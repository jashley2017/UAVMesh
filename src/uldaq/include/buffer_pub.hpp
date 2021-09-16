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

#define MAX_DEV_COUNT 100

namespace uldaq_ros { 
class UldaqPublisher : public rclcpp::Node {
  public:
    explicit UldaqPublisher(const rclcpp::NodeOptions& options);
  private:
    Range getGain(int vRange);
    static void daqEventHandle(DaqDeviceHandle daqDeviceHandle, DaqEventType eventType, unsigned long long eventData, void* userData);
    void _daqEventHandle(DaqDeviceHandle daqDeviceHandle, DaqEventType eventType, unsigned long long eventData, void* userData);
    unsigned long past_scan;
    rclcpp::Publisher<uldaq_msgs::msg::Buffer>::SharedPtr bufpub;
    rclcpp::Publisher<uldaq_msgs::msg::Measurement>::SharedPtr recpub;
    size_t samples_read;
};

struct ScanEventParameters
{
	double* buffer;	// data buffer
  long buffer_size; // size of buffer
	int lowChan;	// first channel in acquisition
	int highChan;	// last channel in acquisition
  UldaqPublisher* node; // node to operate within
};
typedef struct ScanEventParameters ScanEventParameters;
} // namespace uldaq_ros
