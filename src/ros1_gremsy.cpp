#include <gremsy_base/ros1_gremsy.h>

/* GimbalNode //{ */

GimbalNode::GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  // Initialize dynamic-reconfigure
  dynamic_reconfigure::Server<ros1_gremsy::ROSGremsyConfig> server(pnh);
  dynamic_reconfigure::Server<ros1_gremsy::ROSGremsyConfig>::CallbackType f;
  f = boost::bind(&GimbalNode::reconfigureCallback, this, _1, _2);
  server.setCallback(f);

  // Init state variables
  goals_.vector.x = 0;
  goals_.vector.y = 0;
  goals_.vector.z = 0;

  // Advertive Publishers
  encoder_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/ros1_gremsy/encoder", 1000);
  gimbal_attitude_quat_pub_ = nh.advertise<geometry_msgs::Quaternion>("/ros1_gremsy/gimbal_attitude_quaternion", 10);
  gimbal_attitude_euler_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/ros1_gremsy/gimbal_attitude_euler", 10);

  ss_set_gimbal_attitude_ = nh.advertiseService("set_gimbal_attitude", &GimbalNode::setGimbalAttitude, this);

  // Register Subscribers
  gimbal_goal_sub = nh.subscribe("/ros1_gremsy/goals", 1, &GimbalNode::setGoalsCallback, this);

  // Define SDK objects
  serial_port_ = new Serial_Port(config_.device.c_str(), config_.baudrate);

  gimbal_interface_ = new Gimbal_Interface(serial_port_, 1, MAV_COMP_ID_ONBOARD_COMPUTER, Gimbal_Interface::MAVLINK_GIMBAL_V1);

  // Start ther serial interface and the gimbal SDK
  serial_port_->start();
  gimbal_interface_->start();

  ///////////////////
  // Config Gimbal //
  ///////////////////

  // Check if gimbal is on
  if (gimbal_interface_->get_gimbal_status().mode == Gimbal_Interface::GIMBAL_STATE_OFF)
  {
    // Turn on gimbal
    ROS_INFO("TURN_ON!\n");
    gimbal_interface_->set_gimbal_motor(Gimbal_Interface::TURN_ON);
  }

  // Wait until the gimbal is on
  while (!gimbal_interface_->present())
    ros::Duration(0.2).sleep();

  switch (config_.gimbal_mode)
  {
    case 0: {
      ROS_INFO("[%s]: Follow mode chosen ", ros::this_node::getName().c_str());
      gimbal_interface_->set_gimbal_follow_mode_sync();
      break;
    }

    case 1: {
      ROS_INFO("[%s]: Lock mode chosen ", ros::this_node::getName().c_str());
      gimbal_interface_->set_gimbal_lock_mode_sync();
      break;
    }

    default:
      ROS_INFO("[%s]: Undefined mode, shutting down", ros::this_node::getName().c_str());
      gimbal_interface_->set_gimbal_motor(Gimbal_Interface::TURN_OFF);
      ros::shutdown();
  }

  ros::Timer poll_timer = nh.createTimer(ros::Rate(config_.state_poll_rate), &GimbalNode::gimbalStateTimerCallback, this);
  ros::Timer goal_timer = nh.createTimer(ros::Rate(config_.goal_push_rate), &GimbalNode::gimbalGoalTimerCallback, this);
  ros::spin();
}

//}

/* gimbalStateTimerCallback() //{ */

void GimbalNode::gimbalStateTimerCallback(const ros::TimerEvent& event)
{
  // Publish Gimbal Encoder Values
  auto gimbal_encoder_values = gimbal_interface_->get_gimbal_encoder();

  geometry_msgs::Vector3Stamped encoder_ros_msg;
  encoder_ros_msg.header.stamp = ros::Time::now();
  encoder_ros_msg.vector.x = gimbal_encoder_values.pitch;
  encoder_ros_msg.vector.y = gimbal_encoder_values.roll;
  encoder_ros_msg.vector.z = gimbal_encoder_values.yaw;
  // Publish encoder values
  encoder_pub.publish(encoder_ros_msg);

  // Get Mount Orientation
  auto gimbal_attitude = gimbal_interface_->get_gimbal_attitude();
  // Publish Camera Mount Orientation in global frame (drifting)
  // Publish Camera Mount Orientation in global frame (drifting)
  geometry_msgs::Vector3Stamped gimbal_attitude_msg;
  gimbal_attitude_msg.header.stamp = ros::Time::now();
  gimbal_attitude_msg.vector.x = gimbal_attitude.roll;
  gimbal_attitude_msg.vector.y = gimbal_attitude.pitch;
  gimbal_attitude_msg.vector.z = gimbal_attitude.yaw;

  gimbal_attitude_euler_pub_.publish(gimbal_attitude_msg);
  gimbal_attitude_quat_pub_.publish(tf2::toMsg(convertYXZtoQuaternion(gimbal_attitude.roll, gimbal_attitude.pitch, gimbal_attitude.yaw)));
}

//}

/* convertYXZtoQuaternion //{ */

Eigen::Quaterniond GimbalNode::convertYXZtoQuaternion(double roll, double pitch, double yaw)
{
  Eigen::Quaterniond quat_abs(Eigen::AngleAxisd(-DEG_TO_RAD * pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-DEG_TO_RAD * roll, Eigen::Vector3d::UnitX())
                              * Eigen::AngleAxisd(DEG_TO_RAD * yaw, Eigen::Vector3d::UnitZ()));
  return quat_abs;
}

//}

/* gimbalGoalTimerCallback() //{ */

void GimbalNode::gimbalGoalTimerCallback(const ros::TimerEvent& event)
{
  std::scoped_lock lock(mutex_gimbal_);
  gimbal_interface_->set_gimbal_rotation_sync(goals_.vector.x, goals_.vector.y, goals_.vector.z);
}

//}

/* setGimbalAttitude() method //{ */

bool GimbalNode::setGimbalAttitude(ros1_gremsy::SetGimbalAttitude::Request& req, ros1_gremsy::SetGimbalAttitudeResponse& res)
{

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

/* setGoalsCallback() //{ */

void GimbalNode::setGoalsCallback(geometry_msgs::Vector3Stamped message)
{
  ROS_INFO("[%s]: Received goal message ", ros::this_node::getName().c_str());
  std::scoped_lock lock(mutex_gimbal_);
  goals_ = message;
}

//}

/* reconfigureCallback() //{ */

void GimbalNode::reconfigureCallback(ros1_gremsy::ROSGremsyConfig& config, uint32_t level)
{
  config_ = config;
}

//}

/* main() //{ */

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "ros1_gremsy");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  GimbalNode n(nh, pnh);
  return 0;
}

//}
