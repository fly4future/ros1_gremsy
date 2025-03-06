#include <gremsy_base/ros_gremsy.h>

/* GimbalNode //{ */

GimbalNode::GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  // Initialize dynamic-reconfigure
  dynamic_reconfigure::Server<gremsy_base::ROSGremsyConfig> server(pnh);
  dynamic_reconfigure::Server<gremsy_base::ROSGremsyConfig>::CallbackType f;
  f = boost::bind(&GimbalNode::reconfigureCallback, this, _1, _2);
  server.setCallback(f);

  // Init state variables
  goals_.vector.x = 0;
  goals_.vector.y = 0;
  goals_.vector.z = 0;

  // Advertive Publishers
  imu_pub = nh.advertise<sensor_msgs::Imu>("/ros_gremsy/imu/data", 10);
  encoder_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/ros_gremsy/encoder", 1000);
  gimbal_attitude_pub_ = nh.advertise<geometry_msgs::Quaternion>("/ros_gremsy/gimbal_attitude", 10);


  // Register Subscribers
  gimbal_goal_sub = nh.subscribe("/ros_gremsy/goals", 1, &GimbalNode::setGoalsCallback, this);

  // Define SDK objects
  serial_port_ = new Serial_Port(config_.device.c_str(), config_.baudrate);

  gimbal_interface_ = new Gimbal_Interface(serial_port_, 1, MAV_COMP_ID_ONBOARD_COMPUTER, Gimbal_Interface::MAVLINK_GIMBAL_V2, MAVLINK_COMM_0);

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
    }

    default:
      ROS_INFO("[%s]: Undefined mode, shutting down", ros::this_node::getName().c_str());
      gimbal_interface_->set_gimbal_motor(Gimbal_Interface::TURN_OFF);
      ros::shutdown();
  }

  // Set modes for each axis
  /* control_gimbal_axis_mode_t tilt_axis_mode, roll_axis_mode, pan_axis_mode; */

  /* tilt_axis_mode.input_mode = convertIntToAxisInputMode(config_.tilt_axis_input_mode); */
  /* tilt_axis_mode.stabilize = config_.tilt_axis_stabilize; */

  /* roll_axis_mode.input_mode = convertIntToAxisInputMode(config_.roll_axis_input_mode); */
  /* roll_axis_mode.stabilize = config_.roll_axis_stabilize; */

  /* pan_axis_mode.input_mode = convertIntToAxisInputMode(config_.pan_axis_input_mode); */
  /* pan_axis_mode.stabilize = config_.pan_axis_stabilize; */

  /* gimbal_interface_->set_gimbal_axes_mode(tilt_axis_mode, roll_axis_mode, pan_axis_mode); */

  ros::Timer poll_timer = nh.createTimer(ros::Duration(1 / config_.state_poll_rate), &GimbalNode::gimbalStateTimerCallback, this);

  ros::Timer goal_timer = nh.createTimer(ros::Duration(1 / config_.goal_push_rate), &GimbalNode::gimbalGoalTimerCallback, this);

  ros::spin();
}

//}

/* gimbalStateTimerCallback() //{ */

void GimbalNode::gimbalStateTimerCallback(const ros::TimerEvent& event)
{
  // Publish Gimbal IMU
  auto imu_mav = gimbal_interface_->get_gimbal_raw_imu();
  sensor_msgs::Imu imu_ros_mag = convertImuMavlinkMessageToROSMessage(imu_mav);
  imu_pub.publish(imu_ros_mag);

  // Publish Gimbal Encoder Values
  /* mavlink_mount_status_t mount_status = gimbal_interface_->get_gimbal_mount_status(); */
  auto gimbal_encoder_values = gimbal_interface_->get_gimbal_encoder();

  geometry_msgs::Vector3Stamped encoder_ros_msg;
  encoder_ros_msg.header.stamp = ros::Time::now();
  encoder_ros_msg.vector.x = gimbal_encoder_values.pitch;
  encoder_ros_msg.vector.y = gimbal_encoder_values.roll;
  encoder_ros_msg.vector.z = gimbal_encoder_values.yaw;
  //Publish encoder values
  encoder_pub.publish(encoder_ros_msg);

  // Get Mount Orientation
  auto gimbal_attitude = gimbal_interface_->get_gimbal_attitude(); 
  /* mavlink_mount_orientation_t mount_orientation = gimbal_interface_->get_gimbal_mount_orientation(); */

  /* yaw_difference_ = DEG_TO_RAD * (mount_orientation.yaw_absolute - mount_orientation.yaw); */

  // Publish Camera Mount Orientation in global frame (drifting)
  gimbal_attitude_pub_.publish(
      tf2::toMsg(convertYXZtoQuaternion(gimbal_attitude.roll, gimbal_attitude.pitch, gimbal_attitude.yaw)));
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
  /* if (config_.lock_yaw_to_vehicle) */
  /* { */
  /*   z += yaw_difference_; */
  /* } */
  ROS_INFO("[%s]: Moving gimbal..", ros::this_node::getName().c_str());
  gimbal_interface_->set_gimbal_rotation_sync(goals_.vector.x, goals_.vector.y, goals_.vector.z);
}

//}

/* setGoalsCallback() //{ */

void GimbalNode::setGoalsCallback(geometry_msgs::Vector3Stamped message)
{
  /* ROS_INFO("[%s]: Received message ", ros::this_node::getName().c_str()); */
  goals_ = message;
}

//}

/* convertImuMavlinkMessageToROSMessage() //{ */

sensor_msgs::Imu GimbalNode::convertImuMavlinkMessageToROSMessage(Gimbal_Interface::imu_t message)
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

/* /1* convertIntGimbalMode() //{ *1/ */

/* control_gimbal_mode_t GimbalNode::convertIntGimbalMode(int mode) */
/* {  // Allows int access to the control_gimbal_mode_t struct */
/*   switch (mode) */
/*   /1* { *1/ */
/*     case 0: */
/*       return GIMBAL_OFF; */
/*     case 1: */
/*       return LOCK_MODE; */
/*     case 2: */
/*       return FOLLOW_MODE; */
/*     default: */
/*       ROS_ERROR_ONCE("Undefined gimbal mode used. Check the config file."); */
/*       return GIMBAL_OFF; */
/*   } */
/* } */

/* //} */

/* /1* convertIntToAxisInputMode() //{ *1/ */

/* control_gimbal_axis_input_mode_t GimbalNode::convertIntToAxisInputMode(int mode) */
/* {  // Allows int access to the control_gimbal_axis_input_mode_t struct */
/*   switch (mode) */
/*   { */
/*     case 0: */
/*       return CTRL_ANGLE_BODY_FRAME; */
/*     case 1: */
/*       return CTRL_ANGULAR_RATE; */
/*     case 2: */
/*       return CTRL_ANGLE_ABSOLUTE_FRAME; */
/*     default: */
/*       ROS_ERROR_ONCE("Undefined axis input mode used. Check the config file."); */
/*       return CTRL_ANGLE_ABSOLUTE_FRAME; */
/*   } */
/* } */

/* //} */

/* reconfigureCallback() //{ */

void GimbalNode::reconfigureCallback(gremsy_base::ROSGremsyConfig& config, uint32_t level)
{
  config_ = config;
}

//}

/* main() //{ */

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "gremsy_base");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  GimbalNode n(nh, pnh);
  return 0;
}

//}
