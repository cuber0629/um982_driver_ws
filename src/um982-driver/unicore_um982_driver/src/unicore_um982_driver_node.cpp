/**
 * @file unicore_um982_driver_node.cpp
 * @brief ROS 1 driver node for the Unicore UM982 dual-antenna RTK receiver.
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <boost/bind.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "unicore_um982_driver/pvtsln_data.hpp"

class UnicoreDriverNode
{
public:
  UnicoreDriverNode()
    : nh_(),
      private_nh_("~"),
      last_fix_status_("UNKNOWN"),
      last_satellite_count_(0),
      last_data_time_(ros::Time::now())
  {
    private_nh_.param<std::string>("port", port_, "/dev/ttyGPS");
    private_nh_.param<int>("baudrate", baudrate_, 115200);
    private_nh_.getParam("config_commands", config_commands_);

    gps_fix_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>("gps/fix", 10);
    gps_imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("gps/imu", 10);

    diagnostic_updater_.setHardwareID("UM982_GPS");
    diagnostic_updater_.add("RTK GPS", this, &UnicoreDriverNode::diagnosisCallback);

    initializeSerial();
    read_timer_ = nh_.createTimer(ros::Duration(0.05), &UnicoreDriverNode::readSerialData, this);
    diagnostic_timer_ = nh_.createTimer(ros::Duration(1.0), &UnicoreDriverNode::diagnosticTimer, this);

    ROS_INFO("Unicore UM982 ROS 1 driver started on %s at %d baud", port_.c_str(), baudrate_);
  }

  ~UnicoreDriverNode()
  {
    if (serial_.isOpen()) {
      serial_.close();
    }
  }

private:
  void initializeSerial()
  {
    try {
      serial_.setPort(port_);
      serial_.setBaudrate(static_cast<uint32_t>(baudrate_));
      serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
      serial_.setTimeout(timeout);
      serial_.open();
      if (!serial_.isOpen()) {
        throw std::runtime_error("serial port did not open");
      }
      ROS_INFO("Opened serial port %s", port_.c_str());
      configureReceiver();
    } catch (const std::exception& e) {
      ROS_FATAL("Failed to open UM982 serial port %s: %s", port_.c_str(), e.what());
      throw;
    }
  }

  void configureReceiver()
  {
    if (config_commands_.empty()) {
      ROS_WARN("No config_commands parameter supplied; leaving receiver configuration unchanged");
      return;
    }

    for (std::size_t i = 0; i < config_commands_.size(); ++i) {
      const std::string command = config_commands_[i];
      try {
        serial_.write(command + "\r\n");
        ROS_DEBUG("Sent UM982 command %zu/%zu: %s", i + 1, config_commands_.size(), command.c_str());
      } catch (const std::exception& e) {
        ROS_ERROR("Failed to send UM982 command '%s': %s", command.c_str(), e.what());
      }
      ros::Duration(command == "SAVECONFIG" ? 0.5 : 0.1).sleep();
    }
  }

  void readSerialData(const ros::TimerEvent&)
  {
    if (!serial_.isOpen()) {
      return;
    }

    try {
      const std::size_t available = serial_.available();
      if (available == 0) {
        return;
      }
      serial_buffer_ += serial_.read(std::min<std::size_t>(available, 4096));
      processSerialData();
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(1.0, "Error reading UM982 serial data: %s", e.what());
    }
  }

  void processSerialData()
  {
    std::size_t newline = std::string::npos;
    while ((newline = serial_buffer_.find('\n')) != std::string::npos) {
      std::string line = serial_buffer_.substr(0, newline);
      serial_buffer_.erase(0, newline + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (line.find("PVTSLN") == std::string::npos) {
        continue;
      }

      unicore_um982_driver::PVTSLNData data;
      if (!unicore_um982_driver::parsePVTSLN(line, data)) {
        ROS_WARN_THROTTLE(1.0, "Failed to parse PVTSLN message");
        continue;
      }
      publishNavSatFix(data);
      publishImu(data);
    }

    // A disconnected or noisy device must not grow this buffer forever.
    if (serial_buffer_.size() > 16384) {
      serial_buffer_.erase(0, serial_buffer_.size() - 4096);
    }
  }

  void publishNavSatFix(const unicore_um982_driver::PVTSLNData& data)
  {
    sensor_msgs::NavSatFix msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "gps";
    msg.latitude = data.latitude;
    msg.longitude = data.longitude;
    msg.altitude = data.altitude;

    if (data.position_status == "SINGLE") {
      msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    } else if (data.position_status == "RTK_FIXED" || data.position_status == "RTK_FLOAT") {
      msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    } else {
      msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }
    msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    const double east_variance = data.sigma_longitude > 0.0 ? data.sigma_longitude * data.sigma_longitude : 1.0;
    const double north_variance = data.sigma_latitude > 0.0 ? data.sigma_latitude * data.sigma_latitude : 1.0;
    const double up_variance = data.sigma_altitude > 0.0 ? data.sigma_altitude * data.sigma_altitude : 1.0;
    msg.position_covariance[0] = east_variance;
    msg.position_covariance[4] = north_variance;
    msg.position_covariance[8] = up_variance;
    msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    gps_fix_publisher_.publish(msg);
    last_fix_status_ = data.position_status;
    last_satellite_count_ = data.num_satellites_tracked;
    last_data_time_ = msg.header.stamp;
  }

  void publishImu(const unicore_um982_driver::PVTSLNData& data)
  {
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "gps";
    const double heading_rad = data.heading * M_PI / 180.0;
    msg.orientation = headingToQuaternion(heading_rad);
    msg.orientation_covariance[0] = -1.0;
    msg.orientation_covariance[4] = -1.0;
    msg.orientation_covariance[8] = 0.05;
    msg.angular_velocity_covariance[0] = -1.0;
    msg.linear_acceleration_covariance[0] = -1.0;
    gps_imu_publisher_.publish(msg);
  }

  static geometry_msgs::Quaternion headingToQuaternion(double heading_rad)
  {
    geometry_msgs::Quaternion quat;
    quat.z = std::sin(heading_rad / 2.0);
    quat.w = std::cos(heading_rad / 2.0);
    return quat;
  }

  void diagnosticTimer(const ros::TimerEvent&)
  {
    diagnostic_updater_.force_update();
  }

  void diagnosisCallback(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    const double age = (ros::Time::now() - last_data_time_).toSec();
    if (age > 5.0) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No GPS data received");
      stat.add("Status", "NO_DATA");
    } else if (last_fix_status_ == "RTK_FIXED") {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "RTK fix acquired");
    } else if (last_fix_status_ == "RTK_FLOAT" || last_fix_status_ == "DGPS") {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, last_fix_status_);
    } else if (last_fix_status_ == "SINGLE") {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Single point positioning");
    } else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unknown GPS status");
    }
    stat.add("Status", last_fix_status_);
    stat.add("Satellite count", last_satellite_count_);
    stat.add("Data age (s)", age);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher gps_fix_publisher_;
  ros::Publisher gps_imu_publisher_;
  ros::Timer read_timer_;
  ros::Timer diagnostic_timer_;
  diagnostic_updater::Updater diagnostic_updater_;
  serial::Serial serial_;
  std::string port_;
  int baudrate_;
  std::vector<std::string> config_commands_;
  std::string serial_buffer_;
  std::string last_fix_status_;
  int last_satellite_count_;
  ros::Time last_data_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unicore_um982_driver");
  try {
    UnicoreDriverNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("UM982 driver terminated: %s", e.what());
    return 1;
  }
  return 0;
}
