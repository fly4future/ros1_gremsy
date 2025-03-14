/* ros dependencies */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* some STL includes */
#include <stdlib.h>
#include <algorithm>
#include <unistd.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <ros1_gremsy/ROSGremsyConfig.h>
#include <ros1_gremsy/SetGimbalAttitude.h>
#include <ros1_gremsy/GimbalDiagnostics.h>
#include <mrs_msgs/SetInt.h>
#include <cmath>
#include <Eigen/Geometry>
#include <boost/bind.hpp>

// Gimbal API
#include "gimbal_interface.h"
#include "serial_port.h"

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

namespace ros1_gremsy
{
  /* class GremsyDriver //{ */

  class GremsyDriver : public nodelet::Nodelet
  {

  public:
    /* onInit() is called when nodelet is launched (similar to main() in regular node) */
    virtual void onInit();

  private:
    std::atomic<bool> is_initialized_ = false;
    std::atomic<bool> changing_mode_= false;
    /* ros parameters */
    double _rate_timer_controller_ = 60;
    double _state_timer_controller_ = 10;
    std::string _gimbal_sdk_device_id_ = "/dev/ttyUSB0";
    int _gimbal_sdk_baudrate_ = 115200;
    int _gimbal_sdk_mode_ = 2;
    std::mutex mutex_gimbal_;

    /* Gimbal SDK and Serial Port */
    Gimbal_Interface* gimbal_interface_;
    Serial_Port* serial_port_;
    ros1_gremsy::ROSGremsyConfig config_;

    // Goals
    geometry_msgs::Vector3Stamped goals_{};
    std::string gimbal_mode_;
    double gimbal_mount_orientation_yaw = 0;

    // | --------------------- subscribers -------------------- |
    ros::Subscriber goal_sub_;
    // | --------------------- publishers -------------------- |
    ros::Publisher encoder_pub_;
    ros::Publisher gimbal_attitude_pub_quat_;
    ros::Publisher gimbal_attitude_pub_euler_;
    ros::Publisher gimbal_diagnostics_;

    // | --------------------- service server -------------------- |
    ros::ServiceServer ss_set_gimbal_attitude_;
    ros::ServiceServer ss_set_gimbal_mode_;

    // | --------------------- timer callbacks -------------------- |

    void callbackTimerGremsyDriver(const ros::TimerEvent& event);
    void gimbalStateTimerCallback(const ros::TimerEvent& event);
    sensor_msgs::Imu convertImuMavlinkMessageToROSMessage(Gimbal_Interface::imu_t message);
    Eigen::Quaterniond convertYXZtoQuaternion(double roll, double pitch, double yaw);
    void setGoalsCallback(geometry_msgs::Vector3Stamped message);
    void reconfigureCallback(ros1_gremsy::ROSGremsyConfig& config, uint32_t level);
    bool setGimbalAttitude(ros1_gremsy::SetGimbalAttitude::Request& req, ros1_gremsy::SetGimbalAttitude::Response& res);
    bool setGimbalMode(mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& res);
    void publishDiagnostics(void);
    ros::Timer timer_controller_;
    ros::Timer timer_status_;
  };
  //}

  /* onInit() method //{ */

  void GremsyDriver::onInit()
  {
    /* obtain node handle */
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    /* // Initialize dynamic-reconfigure */
    dynamic_reconfigure::Server<ros1_gremsy::ROSGremsyConfig> server(nh);
    dynamic_reconfigure::Server<ros1_gremsy::ROSGremsyConfig>::CallbackType f;
    f = boost::bind(&GremsyDriver::reconfigureCallback, this, _1, _2);
    server.setCallback(f);

    // Load params
    _gimbal_sdk_device_id_ = config_.device;
    _gimbal_sdk_baudrate_ = config_.baudrate;
    _gimbal_sdk_mode_ = config_.gimbal_mode;
    _rate_timer_controller_ = config_.goal_push_rate;
    _state_timer_controller_ = config_.state_poll_rate;

    // | ------------------ initialize subscribers ----------------- |
    // Advertive Publishers
    gimbal_attitude_pub_quat_ = nh.advertise<geometry_msgs::Quaternion>("/ros1_gremsy/gimbal_attitude_quaternion", 1);
    gimbal_attitude_pub_euler_ = nh.advertise<geometry_msgs::Vector3Stamped>("/ros1_gremsy/gimbal_attitude_euler", 1);
    gimbal_diagnostics_ = nh.advertise<ros1_gremsy::GimbalDiagnostics>("/ros1_gremsy/gimbal_diagnostics", 1);

    // Register Subscribers
    goal_sub_ = nh.subscribe("/ros1_gremsy/goals", 1, &GremsyDriver::setGoalsCallback, this);

    ROS_INFO_STREAM("[GremsyDriver]: Config rate: " << config_.goal_push_rate);
    // | -------------------- initialize timers ------------------- |
    timer_controller_ = nh.createTimer(ros::Rate(config_.goal_push_rate), &GremsyDriver::callbackTimerGremsyDriver, this);
    timer_status_ = nh.createTimer(ros::Rate(config_.state_poll_rate), &GremsyDriver::gimbalStateTimerCallback, this);

    ss_set_gimbal_attitude_ = nh.advertiseService("set_gimbal_attitude", &GremsyDriver::setGimbalAttitude, this);
    ss_set_gimbal_mode_ = nh.advertiseService("set_gimbal_mode", &GremsyDriver::setGimbalMode, this);

    /* Define SDK objects */
    ROS_INFO("[GremsyDriver]: Initializing gimbal SDK.");
    serial_port_ = new Serial_Port(_gimbal_sdk_device_id_.c_str(), _gimbal_sdk_baudrate_);
    gimbal_interface_ = new Gimbal_Interface(serial_port_, 1, MAV_COMP_ID_ONBOARD_COMPUTER, Gimbal_Interface::MAVLINK_GIMBAL_V1);

    /* Start ther serial interface and the gimbal SDK */
    ROS_INFO("[GremsyDriver]: Starting serial and gimbal interface.");
    serial_port_->start();
    gimbal_interface_->start();

    /* Wait for the gimbal to be present */
    ROS_WARN("[GremsyDriver]: Waiting for gimbal to be present.");
    while (!gimbal_interface_->present())
    {
      ros::Duration(0.2).sleep();
    }

    /* Config gimbal */
    /* Check if gimbal is on */
    if (gimbal_interface_->get_gimbal_status().mode == Gimbal_Interface::GIMBAL_STATE_OFF)
    {
      ROS_WARN("[GremsyDriver]: Starting the gimbal.");
      gimbal_interface_->set_gimbal_motor(Gimbal_Interface::TURN_ON);
    }

    /* Wait until the gimbal is on */
    ROS_WARN("[GremsyDriver]: Waiting for the gimbal to turn on.");
    while (gimbal_interface_->get_gimbal_status().mode < Gimbal_Interface::GIMBAL_STATE_ON)
    {
      ros::Duration(0.2).sleep();
    }

    switch (_gimbal_sdk_mode_)
    {
      case 1: {
        ROS_WARN("[GremsyDriver]: Setting gimbal to lock mode.");
        gimbal_interface_->set_gimbal_lock_mode_sync();
        gimbal_mode_ = "lock";
        break;
      }
      case 2: {
        ROS_WARN("[GremsyDriver]: Setting gimbal to follow mode.");
        gimbal_interface_->set_gimbal_follow_mode_sync();
        gimbal_mode_ = "follow";
        break;
      }
      default: {
        ROS_ERROR("[GremsyDriver]: Incorrect gimbal mode. Check config file. Shutting down node.");
        ros::shutdown();
      }
    }
    ROS_INFO_ONCE("[GremsyDriver]: Node initialized");
    is_initialized_ = true;
  }

  //}

  /* setGimbalAttitude() method //{ */

  bool GremsyDriver::setGimbalAttitude(ros1_gremsy::SetGimbalAttitude::Request& req, ros1_gremsy::SetGimbalAttitudeResponse& res)
  {

    if (!is_initialized_)
    {
      res.success = false;
      res.message = "GimbalController not initialized.";
      return true;
    }

    ROS_INFO("[GimbalController]: Attitude request received - Roll: %f, Pitch: %f, Yaw: %f", req.roll, req.pitch, req.yaw);
    std::scoped_lock lock(mutex_gimbal_);
    {
      goals_.vector.x = req.pitch;
      goals_.vector.y = req.roll;
      goals_.vector.z = req.yaw;
    }

    res.success = true;
    res.message = "Gimbal orientation set successfully.";
    return true;
  }
  //}

  /* setGimbalMode() //{ */

  bool GremsyDriver::setGimbalMode(mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& res)
  {
    if (!is_initialized_)
    {
      res.success = false;
      res.message = "GimbalController not initialized.";
      return true;
    }
    changing_mode_ = true;
    bool success = true;
    std::stringstream ss;
    ROS_INFO("[GimbalController]: Change operation mode request received - Value: %f", req.value);
    {
      std::scoped_lock lock(mutex_gimbal_);
      switch (req.value)
      {
        case 1: {
          if (gimbal_mode_ != "lock")
          {
            ROS_INFO("[GremsyDriver]: Setting gimbal to lock mode.");
            gimbal_interface_->set_gimbal_lock_mode_sync();
            gimbal_mode_ = "lock";
            ss << "Changed mode successfully into lock mode";
          } else
          {
            ROS_WARN("[GremsyDriver]: Gimbal already in lock mode. Ignoring...");
            success = false;
            ss << "Gimbal already in follow mode";
          }
          break;
        }
        case 2: {
          if (gimbal_mode_ != "follow")
          {
            ROS_INFO("[GremsyDriver]: Setting gimbal to follow mode.");
            gimbal_interface_->set_gimbal_follow_mode_sync();
            gimbal_mode_ = "follow";
            ss << "Changed mode successfully into follow mode";
          } else
          {
            ROS_WARN("[GremsyDriver]: Gimbal already in follow mode. Ignoring...");
            success = false;
            ss << "Gimbal already in follow mode";
          }
          break;
        }
        default: {
          ROS_WARN("[GremsyDriver]: Incorrect gimbal mode. Gimbal only supports (lock) and (follow).");
          success = false;
          ss << "Incorrect gimbal mode. Gimbal only supports (lock) and (follow).";
          break;
        }
      }
    }
    res.success = success;
    res.message = ss.str();
    changing_mode_ = false;
    return true;
  }

  //}

  /* callbackTimerGremsyDriver() method //{ */

  void GremsyDriver::callbackTimerGremsyDriver(const ros::TimerEvent& event)
  {
    if (!is_initialized_ || changing_mode_)
    {
      return;
    }
    // Gimbal expects the arguments in the following order: pitch,roll,yaw
    std::scoped_lock lock(mutex_gimbal_);
    gimbal_interface_->set_gimbal_rotation_sync(goals_.vector.x, goals_.vector.y, goals_.vector.z * (-1));
  }

  //}

  /* gimbalStateTimerCallback() //{ */

  void GremsyDriver::gimbalStateTimerCallback(const ros::TimerEvent& event)
  {
    if (!is_initialized_)
    {
      return;
    }
    // Gremsy follows North-East-Down (NED) coordinates convention, we are using East-North-Up (ENU) convention
    // NED: X-axis/Roll: Forward, Y-axis/Pitch: Right , Z-axis/Yaw: Down
    // ENU: Y-axis/Roll: Forward, X-axis/Pitch: Left, Z-axis/Yaw: Up
    // Get Mount Orientation
    auto gimbal_attitude = gimbal_interface_->get_gimbal_attitude();
    // Publish Camera Mount Orientation in global frame
    geometry_msgs::Vector3Stamped gimbal_attitude_msg;
    gimbal_attitude_msg.header.stamp = ros::Time::now();
    gimbal_attitude_msg.vector.x = gimbal_attitude.pitch;
    gimbal_attitude_msg.vector.y = gimbal_attitude.roll;
    gimbal_attitude_msg.vector.z = gimbal_attitude.yaw * (-1);

    ROS_INFO_STREAM("[GremsyDriver]: Yaw is : "<< gimbal_attitude_msg.vector.z);

    auto gimbal_attitude_quaternion_msg =
        tf2::toMsg(convertYXZtoQuaternion(gimbal_attitude_msg.vector.y, gimbal_attitude_msg.vector.x, gimbal_attitude_msg.vector.z));

    gimbal_attitude_pub_euler_.publish(gimbal_attitude_msg);
    gimbal_attitude_pub_quat_.publish(
        tf2::toMsg(convertYXZtoQuaternion(gimbal_attitude_msg.vector.y, gimbal_attitude_msg.vector.x, gimbal_attitude_msg.vector.z)));

    ros1_gremsy::GimbalDiagnostics diagnostics_msg;
    diagnostics_msg.stamp = ros::Time::now();
    diagnostics_msg.attitude_euler = gimbal_attitude_msg;
    diagnostics_msg.attitude_quaternion = gimbal_attitude_quaternion_msg;

    std::scoped_lock lock(mutex_gimbal_);
    {
      diagnostics_msg.setpoint = goals_;
      diagnostics_msg.mode = gimbal_mode_;
    }

    gimbal_diagnostics_.publish(diagnostics_msg);
  }

  //}

  /* convertImuMavlinkMessageToROSMessage() //{ */

  sensor_msgs::Imu GremsyDriver::convertImuMavlinkMessageToROSMessage(Gimbal_Interface::imu_t message)
  {
    sensor_msgs::Imu imu_message;

    // Set accelaration data
    imu_message.linear_acceleration.x = message.accel.x;
    imu_message.linear_acceleration.y = message.accel.y;
    imu_message.linear_acceleration.z = message.accel.z;

    // Set gyro data
    imu_message.angular_velocity.x = message.gyro.x;
    imu_message.angular_velocity.y = message.gyro.y;
    imu_message.angular_velocity.z = message.gyro.z;
    ROS_INFO("[GremsyDriver]: Processed imu message");

    return imu_message;
  }

  //}

  /* convertYXZtoQuaternion //{ */

  Eigen::Quaterniond GremsyDriver::convertYXZtoQuaternion(double roll, double pitch, double yaw)
  {
    Eigen::Quaterniond quat_abs(Eigen::AngleAxisd(-DEG_TO_RAD * pitch, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(-DEG_TO_RAD * roll, Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(DEG_TO_RAD * yaw, Eigen::Vector3d::UnitZ()));
    return quat_abs;
  }

  //}

  /* setGoalsCallback() //{ */

  void GremsyDriver::setGoalsCallback(geometry_msgs::Vector3Stamped message)
  {
    ROS_INFO("[%s]: Received message ", ros::this_node::getName().c_str());
    std::scoped_lock lock(mutex_gimbal_);
    goals_ = message;
  }

  //}

  /* reconfigureCallback() //{ */

  void GremsyDriver::reconfigureCallback(ros1_gremsy::ROSGremsyConfig& config, uint32_t level)
  {
    config_ = config;
  }

  //}
};  // namespace ros1_gremsy

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros1_gremsy::GremsyDriver, nodelet::Nodelet);
