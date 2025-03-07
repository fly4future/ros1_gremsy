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
#include <ros_gremsy/ROSGremsyConfig.h>
#include <ros_gremsy/SetGimbalAttitude.h>
#include <cmath>
#include <Eigen/Geometry>
#include <boost/bind.hpp>

// Gimbal API
#include "gimbal_interface.h"
#include "serial_port.h"

/* #include <ros_gremsy/> */

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

namespace ros_gremsy
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

    /* Gimbal SDK */
    Gimbal_Interface* gimbal_interface_;
    /* Serial Interface */
    Serial_Port* serial_port_;
    gremsy_base::ROSGremsyConfig config_;

    // Goals
    geometry_msgs::Vector3Stamped goals_;

    // | --------------------- subscribers -------------------- |
    ros::Subscriber goal_sub_;
    // | --------------------- publishers -------------------- |
    ros::Publisher imu_pub_;
    ros::Publisher encoder_pub_;
    ros::Publisher gimbal_attitude_pub_;

    // | --------------------- service server -------------------- |
    ros::ServiceServer ss_set_gimbal_attitude_;

    // | --------------------- timer callbacks -------------------- |

    void callbackTimerGremsyDriver(const ros::TimerEvent& event);
    void gimbalStateTimerCallback(const ros::TimerEvent& event);
    sensor_msgs::Imu convertImuMavlinkMessageToROSMessage(Gimbal_Interface::imu_t message);
    Eigen::Quaterniond convertYXZtoQuaternion(double roll, double pitch, double yaw);
    void setGoalsCallback(geometry_msgs::Vector3Stamped message);
    void reconfigureCallback(gremsy_base::ROSGremsyConfig& config, uint32_t level);
    bool setGimbalAttitude(ros_gremsy::SetGimbalAttitude::Request& req, ros_gremsy::SetGimbalAttitude::Response& res);
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
    dynamic_reconfigure::Server<gremsy_base::ROSGremsyConfig> server(nh);
    dynamic_reconfigure::Server<gremsy_base::ROSGremsyConfig>::CallbackType f;
    f = boost::bind(&GremsyDriver::reconfigureCallback, this, _1, _2);
    server.setCallback(f);

    // | ------------------ initialize subscribers ----------------- |
    // Advertive Publishers
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("/ros_gremsy/imu/data", 10);
    encoder_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/ros_gremsy/encoder", 1000);
    gimbal_attitude_pub_ = nh.advertise<geometry_msgs::Quaternion>("/ros_gremsy/gimbal_attitude", 10);

    // Register Subscribers
    goal_sub_ = nh.subscribe("/ros_gremsy/goals", 1, &GremsyDriver::setGoalsCallback, this);

    // | -------------------- initialize timers ------------------- |
    timer_controller_ = nh.createTimer(ros::Rate(_rate_timer_controller_), &GremsyDriver::callbackTimerGremsyDriver, this);
    /* timer_status_ = nh.createTimer(ros::Rate(_rate_timer_controller_), &GremsyDriver::gimbalStateTimerCallback, this); */


    ss_set_gimbal_attitude_ = nh.advertiseService("set_gimbal_manual_control", &GremsyDriver::setGimbalAttitude, this);

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

  bool GremsyDriver::setGimbalAttitude(ros_gremsy::SetGimbalAttitude::Request& req, ros_gremsy::SetGimbalAttitudeResponse& res)
  {

    if (!is_initialized_)
    {
      res.success = false;
      res.message = "GimbalController not initialized.";
      return true;
    }
    auto gimbal_encoder_values = gimbal_interface_->get_gimbal_encoder();

    ROS_INFO_STREAM("[GimbalController]: Gimbal encoder before setting.\n Roll: " << static_cast<double>(gimbal_encoder_values.roll)
                                                                                  << ", Pitch: " << static_cast<double>(gimbal_encoder_values.pitch * -1)
                                                                                  << ", Yaw: " << static_cast<double>(gimbal_encoder_values.yaw) << ".");

    ROS_INFO("[GimbalController]: Manual control request received - Pitch: %f, Roll: %f, Yaw: %f", req.pitch, req.roll, req.yaw);

    ROS_WARN_STREAM("[GimbalController]: Gimbal command.\n Roll: " << static_cast<double>(req.roll) << ", Pitch: " << static_cast<double>(req.pitch)
                                                                   << ", Yaw: " << static_cast<double>(req.yaw) << ".");

    goals_.vector.x = req.pitch;
    goals_.vector.y = req.roll;
    goals_.vector.z = req.yaw;

    gimbal_encoder_values = gimbal_interface_->get_gimbal_encoder();

    ROS_INFO_STREAM("[GimbalController]: Gimbal encoder values after setting.\n Roll: " << static_cast<double>(gimbal_encoder_values.roll)
                                                                                        << ", Pitch: " << static_cast<double>(gimbal_encoder_values.pitch * -1)
                                                                                        << ", Yaw: " << static_cast<double>(gimbal_encoder_values.yaw) << ".");


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
    gimbal_interface_->set_gimbal_rotation_sync(goals_.vector.x, goals_.vector.y, goals_.vector.z);
  }

  //}

  /* gimbalStateTimerCallback() //{ */

  void GremsyDriver::gimbalStateTimerCallback(const ros::TimerEvent& event)
  {

    //This doesn't work with new version/ gets blocked 

    if (!is_initialized_)
    {
      return;
    }
    ROS_INFO("[%s]: Processing gimbal data", ros::this_node::getName().c_str());
    // Publish Gimbal IMU
    auto imu_mav = gimbal_interface_->get_gimbal_raw_imu();
    ROS_INFO_STREAM("[GremsyDriver]: imu_mav " << imu_mav.accel.x);
    sensor_msgs::Imu imu_ros_mag = convertImuMavlinkMessageToROSMessage(imu_mav);
    imu_pub_.publish(imu_ros_mag);
    ROS_INFO("[GremsyDriver]: Published imu");

    // Publish Gimbal Encoder Values
    /* mavlink_mount_status_t mount_status = gimbal_interface_->get_gimbal_mount_status(); */
    auto gimbal_encoder_values = gimbal_interface_->get_gimbal_encoder();

    ROS_INFO_STREAM("[GremsyDriver]: gimbal encoder: pitch:" << gimbal_encoder_values.pitch << "yaw: " << gimbal_encoder_values.yaw
                                                             << "roll: " << gimbal_encoder_values.roll);

    geometry_msgs::Vector3Stamped encoder_ros_msg;
    encoder_ros_msg.header.stamp = ros::Time::now();
    encoder_ros_msg.vector.x = gimbal_encoder_values.pitch;
    encoder_ros_msg.vector.y = gimbal_encoder_values.roll;
    encoder_ros_msg.vector.z = gimbal_encoder_values.yaw;
    // Publish encoder values
    encoder_pub_.publish(encoder_ros_msg);
    ROS_INFO("[GremsyDriver]: Getting attitude");

    // Get Mount Orientation
    auto gimbal_attitude = gimbal_interface_->get_gimbal_attitude();
    ROS_INFO("[GremsyDriver]: Got attitude");
    /* mavlink_mount_orientation_t mount_orientation = gimbal_interface_->get_gimbal_mount_orientation(); */

    /* yaw_difference_ = DEG_TO_RAD * (mount_orientation.yaw_absolute - mount_orientation.yaw); */

    /* // Publish Camera Mount Orientation in global frame (drifting) */
    gimbal_attitude_pub_.publish(tf2::toMsg(convertYXZtoQuaternion(gimbal_attitude.roll, gimbal_attitude.pitch, gimbal_attitude.yaw)));
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
    goals_ = message;
  }

  //}

  /* reconfigureCallback() //{ */

  void GremsyDriver::reconfigureCallback(gremsy_base::ROSGremsyConfig& config, uint32_t level)
  {
    config_ = config;
  }

  //}
};  // namespace ros_gremsy

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_gremsy::GremsyDriver, nodelet::Nodelet);
