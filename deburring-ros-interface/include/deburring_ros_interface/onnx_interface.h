#ifndef DEBURRING_ROS_INTERFACE
#define DEBURRING_ROS_INTERFACE

#include <linear_feedback_controller_msgs/Sensor.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>

class DeburringONNXInterface {
 public:
  DeburringONNXInterface(ros::NodeHandle nh, const size_t state_size,
                         const size_t horizon_size);
  void update(const Eigen::VectorXd& x0, const std::vector<Eigen::VectorXd>& X,
              const Eigen::Vector3d& target_pos);

  const Eigen::VectorXd& getReferenceState();
  std_msgs::Float64MultiArray& get_nnOutput() { return nn_output_; }

 private:
  void nnCb(const std_msgs::Float64MultiArrayConstPtr msg);

  const size_t state_size_;
  const size_t horizon_size_;
  std::vector<size_t> rl_controlled_ids;

  ros::Subscriber nn_sub_;
  ros::Publisher nn_pub_;

  // prealocated memory
  Eigen::VectorXd xref_;
  std_msgs::Float64MultiArray nn_output_;

  size_t size_input_vector_;
  Eigen::VectorXd x0_;
  Eigen::VectorXd nn_input_vector_;
  std_msgs::Float64MultiArray nn_input_msg_;
};

#endif  // DEBURRING_ROS_INTERFACE
