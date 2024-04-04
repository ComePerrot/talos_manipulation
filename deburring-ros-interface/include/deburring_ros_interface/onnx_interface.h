#ifndef DEBURRING_ROS_INTERFACE
#define DEBURRING_ROS_INTERFACE

#include <linear_feedback_controller_msgs/Sensor.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>

class DeburringONNXInterface {
 public:
  DeburringONNXInterface(ros::NodeHandle nh);
  void update(const Eigen::VectorXd& x0, const std::vector<Eigen::VectorXd>& X,
              const Eigen::Vector3d& target_pos);

 private:
  void nnCb(const std_msgs::Float64MultiArrayConstPtr msg);

  ros::Subscriber nn_sub_;
  ros::Publisher nn_pub_;

  std_msgs::Float64MultiArray nn_output_;
};

#endif  // DEBURRING_ROS_INTERFACE
