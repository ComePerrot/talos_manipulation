#include "deburring_ros_interface/onnx_interface.h"
#include <eigen_conversions/eigen_msg.h>

DeburringONNXInterface::DeburringONNXInterface(ros::NodeHandle nh,
                                               const size_t state_size,
                                               const size_t horizon_size)
    : state_size_(state_size), horizon_size_(horizon_size) {
  x0_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(state_size));
  size_input_vector_ = state_size * 4 + 3;
  nn_input_vector_.resize(static_cast<Eigen::Index>(size_input_vector_));

  // NN action subscriber
  nn_sub_ =
      nh.subscribe("/scaled_action", 1, &DeburringONNXInterface::nnCb, this);

  // NN input publisher
  nn_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/nn_input", 1);
}

void DeburringONNXInterface::nnCb(
    const std_msgs::Float64MultiArrayConstPtr msg) {
  nn_output_ = *msg;
}

void DeburringONNXInterface::update(const Eigen::VectorXd& x0,
                                    const std::vector<Eigen::VectorXd>& X,
                                    const Eigen::Vector3d& target_pos) {
  
  nn_input_vector_ << x0, X[0], X[1], X[2], target_pos;
  tf::matrixEigenToMsg(nn_input_vector_, nn_input_msg_);

  // Publish the nn_input_msg
  nn_pub_.publish(nn_input_msg_);
}
