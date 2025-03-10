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
#include "gimbal_interface.h"
#include "serial_port.h"

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)


class GimbalNode
{
public:
  // Params: (public node handler (for e.g. callbacks), private node handle (for e.g. dynamic reconfigure))
  GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
  // Dynamic reconfigure callback
  void reconfigureCallback(ros1_gremsy::ROSGremsyConfig& config, uint32_t level);
  // Timer which checks for new infomation regarding the gimbal
  void gimbalStateTimerCallback(const ros::TimerEvent& event);
  // Timer which sets the gimbal goals
  void gimbalGoalTimerCallback(const ros::TimerEvent& event);
  // Calback to set a new gimbal position
  void setGoalsCallback(geometry_msgs::Vector3Stamped message);

  bool setGimbalAttitude(ros1_gremsy::SetGimbalAttitude::Request& req, ros1_gremsy::SetGimbalAttitude::Response& res);

  // Converts
  Eigen::Quaterniond convertYXZtoQuaternion(double roll, double pitch, double yaw);

  // Gimbal SDK
  Gimbal_Interface* gimbal_interface_;
  // Serial Interface
  Serial_Port* serial_port_;
  // Current config
  ros1_gremsy::ROSGremsyConfig config_;
  // Publishers
  ros::Publisher imu_pub, encoder_pub, gimbal_attitude_quat_pub_,gimbal_attitude_euler_pub_, mount_orientation_incl_local_yaw;
  // Subscribers
  ros::Subscriber gimbal_goal_sub;

  //Service server
  ros::ServiceServer ss_set_gimbal_attitude_;
  // Value store
  geometry_msgs::Vector3Stamped goals_;
  std::mutex mutex_gimbal_;
};

