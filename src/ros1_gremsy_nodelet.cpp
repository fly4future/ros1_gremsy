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
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <ros1_gremsy/ROSGremsyConfig.h>
#include <ros1_gremsy/SetGimbalAttitude.h>
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
    bool is_initialized_ = false;
    /* ros parameters */
    int _rate_timer_controller_ = 100;
    std::string _gimbal_sdk_device_id_ = "/dev/gimbal";
    int _gimbal_sdk_baudrate_ = 115200;
    std::string _gimbal_sdk_mode_ = "follow";

    std::mutex mutex_gimbal_;

    /* Gimbal SDK */
    Gimbal_Interface* gimbal_interface_;
    /* Serial Interface */
    Serial_Port* serial_port_;
    ros1_gremsy::ROSGremsyConfig config_;

    // Goals
    geometry_msgs::Vector3Stamped goals_;

    // | --------------------- subscribers -------------------- |
    ros::Subscriber goal_sub_;
    // | --------------------- publishers -------------------- |
    ros::Publisher imu_pub_;
    ros::Publisher encoder_pub_;
    ros::Publisher gimbal_attitude_pub_quat_;
    ros::Publisher gimbal_attitude_pub_euler_;

    // | --------------------- service server -------------------- |
    ros::ServiceServer ss_set_gimbal_attitude_;

    // | --------------------- timer callbacks -------------------- |

    void callbackTimerGremsyDriver(const ros::TimerEvent& event);
    void gimbalStateTimerCallback(const ros::TimerEvent& event);
    sensor_msgs::Imu convertImuMavlinkMessageToROSMessage(Gimbal_Interface::imu_t message);
    Eigen::Quaterniond convertYXZtoQuaternion(double roll, double pitch, double yaw);
    void setGoalsCallback(geometry_msgs::Vector3Stamped message);
    void reconfigureCallback(ros1_gremsy::ROSGremsyConfig& config, uint32_t level);
    bool setGimbalAttitude(ros1_gremsy::SetGimbalAttitude::Request& req, ros1_gremsy::SetGimbalAttitude::Response& res);
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

    // | ------------------ initialize subscribers ----------------- |
    // Advertive Publishers
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("/ros1_gremsy/imu/data", 10);
    encoder_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/ros1_gremsy/encoder", 1000);
    gimbal_attitude_pub_quat_ = nh.advertise<geometry_msgs::Quaternion>("/ros1_gremsy/gimbal_attitude_quaternion", 10);
    gimbal_attitude_pub_euler_ = nh.advertise<geometry_msgs::Vector3Stamped>("/ros1_gremsy/gimbal_attitude_euler", 10);

    // Register Subscribers
    goal_sub_ = nh.subscribe("/ros1_gremsy/goals", 1, &GremsyDriver::setGoalsCallback, this);

    // | -------------------- initialize timers ------------------- |
    timer_controller_ = nh.createTimer(ros::Rate(60), &GremsyDriver::callbackTimerGremsyDriver, this);
    timer_status_ = nh.createTimer(ros::Rate(10), &GremsyDriver::gimbalStateTimerCallback, this);


    ss_set_gimbal_attitude_ = nh.advertiseService("set_gimbal_attitude", &GremsyDriver::setGimbalAttitude, this);

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

    /*        To avoid found bug of PixyLR with SDK           */
    Gimbal_Interface::gimbal_config_axis_t tilt_setting = gimbal_interface_->get_gimbal_config_tilt_axis();

    gimbal_interface_->set_gimbal_config_tilt_axis(tilt_setting);

    Gimbal_Interface::gimbal_config_axis_t pan_setting = gimbal_interface_->get_gimbal_config_pan_axis();

    gimbal_interface_->set_gimbal_config_tilt_axis(pan_setting);

    Gimbal_Interface::gimbal_config_axis_t roll_setting = gimbal_interface_->get_gimbal_config_roll_axis();

    gimbal_interface_->set_gimbal_config_roll_axis(roll_setting);


    // gimbal_interface_->set_gimbal_config_tilt_axis();
    /* Set gimbal control modes */
    if (_gimbal_sdk_mode_ == "follow")
    {

      ROS_WARN("[GremsyDriver]: Setting gimbal to follow mode.");
      gimbal_interface_->set_gimbal_follow_mode_sync();
    } else
    {

      if (_gimbal_sdk_mode_ == "lock")
      {

        ROS_WARN("[GremsyDriver]: Setting gimbal to lock mode.");
        gimbal_interface_->set_gimbal_lock_mode_sync();
      } else
      {

        ROS_ERROR("[GremsyDriver]: Incorrect gimbal mode. Check config file. Shutting down node.");
        ros::shutdown();
      }
    }

    /* Configure the gimbal to send angles as encoder values */
    gimbal_interface_->set_gimbal_encoder_type_send(false);


    /*     // Ask afzal why this is here */
    /*     // timer_controller_ = nh.createTimer(ros::Rate(_rate_timer_controller_), &GremsyDriver::callbackTimerGremsyDriver, this); */

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
    std::scoped_lock lock(mutex_gimbal_);
    auto gimbal_encoder_values = gimbal_interface_->get_gimbal_encoder();

    ROS_INFO("[GimbalController]: Attitude request received - Pitch: %f, Roll: %f, Yaw: %f", req.pitch, req.roll, req.yaw);

    goals_.vector.x = req.pitch;
    goals_.vector.y = req.roll;
    goals_.vector.z = req.yaw;

    res.success = true;
    res.message = "Gimbal orientation set successfully.";
    return true;
  }
  //}

  /* callbackTimerGremsyDriver() method //{ */

  void GremsyDriver::callbackTimerGremsyDriver(const ros::TimerEvent& event)
  {
    if (!is_initialized_)
    {
      return;
    }
    std::scoped_lock lock(mutex_gimbal_);
    gimbal_interface_->set_gimbal_rotation_sync(goals_.vector.x, goals_.vector.y, goals_.vector.z);
  }

  //}

  /* gimbalStateTimerCallback() //{ */

  void GremsyDriver::gimbalStateTimerCallback(const ros::TimerEvent& event)
  {

    // This doesn't work with new version/ gets blocked

    if (!is_initialized_)
    {
      return;
    }
    // Publish Gimbal Encoder Values
    auto gimbal_encoder_values = gimbal_interface_->get_gimbal_encoder();

    geometry_msgs::Vector3Stamped encoder_ros_msg;
    encoder_ros_msg.header.stamp = ros::Time::now();
    encoder_ros_msg.vector.x = gimbal_encoder_values.pitch;
    encoder_ros_msg.vector.y = gimbal_encoder_values.roll;
    encoder_ros_msg.vector.z = gimbal_encoder_values.yaw;
    // Publish encoder values
    encoder_pub_.publish(encoder_ros_msg);

    // Get Mount Orientation
    auto gimbal_attitude = gimbal_interface_->get_gimbal_attitude();

    // Publish Camera Mount Orientation in global frame (drifting)
    geometry_msgs::Vector3Stamped gimbal_attitude_msg;
    gimbal_attitude_msg.header.stamp = ros::Time::now();
    gimbal_attitude_msg.vector.x = gimbal_attitude.roll;
    gimbal_attitude_msg.vector.y = gimbal_attitude.pitch;
    gimbal_attitude_msg.vector.z = gimbal_attitude.yaw;

    gimbal_attitude_pub_euler_.publish(gimbal_attitude_msg);
    gimbal_attitude_pub_quat_.publish(tf2::toMsg(convertYXZtoQuaternion(gimbal_attitude.roll, gimbal_attitude.pitch, gimbal_attitude.yaw)));
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
