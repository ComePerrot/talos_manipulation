#include "deburring_ros_interface/onnx_interface.h"

#include <eigen_conversions/eigen_msg.h>

DeburringONNXInterface::DeburringONNXInterface(ros::NodeHandle nh,
                                               const size_t state_size,
                                               const size_t horizon_size)
    : state_size_(state_size), horizon_size_(horizon_size) {
  x0_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(state_size));
  xref_.resize(static_cast<Eigen::Index>(state_size));
  size_input_vector_ = state_size * 4 + 3;
  nn_input_vector_.resize(static_cast<Eigen::Index>(size_input_vector_));

  rl_controlled_ids = {0, 1, 2};

  // Neural Network action subscriber
  nn_sub_ =
      nh.subscribe("/scaled_action", 1, &DeburringONNXInterface::nnCb, this);

  // Neural Network input publisher
  nn_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/nn_input", 1);
}

void DeburringONNXInterface::nnCb(
    const std_msgs::Float64MultiArrayConstPtr msg) {
  nn_output_ = *msg;
}

void DeburringONNXInterface::update(const Eigen::VectorXd& x0,
                                    const std::vector<Eigen::VectorXd>& X,
                                    const Eigen::Vector3d& target_pos) {
  x0_ << x0;
  nn_input_vector_ << x0, X[0], X[1], X[2],
      target_pos;  /// @TODO choose the rigth element of X
  tf::matrixEigenToMsg(nn_input_vector_, nn_input_msg_);

  // Publish the nn_input_msg
  nn_pub_.publish(nn_input_msg_);
}

const Eigen::VectorXd& DeburringONNXInterface::getReferenceState() {
  xref_ << x0_;

  for (size_t i = 0; i < rl_controlled_ids.size(); ++i) {
    xref_[static_cast<Eigen::Index>(rl_controlled_ids[i])] = nn_output_.data[i];
  }

  return (xref_);
}
