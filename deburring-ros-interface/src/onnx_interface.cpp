#include "deburring_ros_interface/onnx_interface.h"

#include <eigen_conversions/eigen_msg.h>

DeburringONNXInterface::DeburringONNXInterface(ros::NodeHandle nh,
                                               const size_t state_size,
                                               const size_t horizon_size)
    : state_size_(state_size), horizon_size_(horizon_size) {
  setupParameters();
  x0_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(state_size_));
  xref_.resize(static_cast<Eigen::Index>(state_size));
  nn_input_vector_ =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(size_input_vector_));

  // Neural Network action subscriber
  nn_sub_ =
      nh.subscribe("/scaled_action", 1, &DeburringONNXInterface::nnCb, this);

  // Neural Network input publisher
  nn_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/nn_input", 1);

  ros::Rate r(1);  // Rate for reading inital state from the robot
  while (nn_output_.data.empty()) {
    // No measurments have been received if message time stamp is zero
    ROS_INFO_STREAM("Waiting for neural network action");
    ros::spinOnce();

    r.sleep();
  }
}

void DeburringONNXInterface::setupParameters() {
  //  Observations
  observed_state_ids_ = {33, 66, 100};
  size_input_vector_ = state_size_ * (observed_state_ids_.size() + 1) + 3;
  average_state_.resize(57);
  amplitude_state_.resize(57);
  // average_state_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.61086524, 0.0,
  // -0.6975,
  //     1.309, -0.295, 0.0, -0.61086524, 0.0, -0.6975, 1.309, -0.295, 0.0, 0.0,
  //     0.25307274, -0.39269908, 1.43989663, 0.0, -1.11526539, 0.39269908,
  //     -1.43989663, 0.0, -1.11526539, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // amplitude_state_ << 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 1.74532925, 0.7854,
  //     1.7475, 2.618, 1.315, 0.7854, 1.13446401, 0.7854, 1.7475, 2.618, 1.315,
  //     0.7854, 1.88495559, 0.84648469, 1.57079633, 2.8667033, 3.63901149,
  //     1.12050138, 1.96349541, 1.42680666, 3.63901149, 1.12050138, 7.5, 7.5, 7.5,
  //     7.5, 7.5, 7.5, 5.805, 8.7, 8.7, 10.5, 8.7, 7.2, 5.805, 8.7, 8.7, 10.5,
  //     8.7, 7.2, 8.1, 8.1, 4.05, 5.49, 6.87, 6.87, 4.05, 5.49, 6.87, 6.87;
  // average_target_ << 0.6, 0.4, 1.1;
  // amplitude_target << 0.45, 0.35, 0.7;

  //  Action
  rl_controlled_ids_ = {21, 22, 24};
  action_scale_ = 0.5;
  action_amplitude_.resize(
      static_cast<Eigen::Index>(rl_controlled_ids_.size()));
  action_amplitude_ << 1.17809725, 1.43116999, 1.11875605;

  xref_.resize(static_cast<Eigen::Index>(state_size_));
  nn_input_vector_.resize(static_cast<Eigen::Index>(size_input_vector_));
}

void DeburringONNXInterface::nnCb(
    const std_msgs::Float64MultiArrayConstPtr msg) {
  nn_output_ = *msg;
}

void DeburringONNXInterface::update(const Eigen::VectorXd& x0,
                                    const std::vector<Eigen::VectorXd>& X,
                                    const Eigen::Vector3d& target_pos) {
  x0_ << x0;

  buildInputVector(x0, X, target_pos);
  // normalizeObservations(x0, X, target_pos);

  tf::matrixEigenToMsg(nn_input_vector_, nn_input_msg_);

  // Publish the nn_input_msg
  nn_pub_.publish(nn_input_msg_);
}

void DeburringONNXInterface::buildInputVector(
    const Eigen::VectorXd& x0, const std::vector<Eigen::VectorXd>& X,
    const Eigen::Vector3d& target_pos) {
  nn_input_vector_.head(static_cast<Eigen::Index>(state_size_)) = x0;
  nn_input_vector_.segment(static_cast<Eigen::Index>(state_size_), 3) =
      target_pos;

  for (size_t i = 0; i < observed_state_ids_.size(); ++i) {
    nn_input_vector_.segment(
        static_cast<Eigen::Index>(state_size_ + 3 + i * state_size_),
        static_cast<Eigen::Index>(state_size_)) = X[observed_state_ids_[i]];
  }
}

// void DeburringONNXInterface::normalizeObservations(
//     const Eigen::VectorXd& x0, const std::vector<Eigen::VectorXd>& X,
//     const Eigen::Vector3d& target_pos) {
//   nn_input_vector_ << (x0 - average_state_).cwiseQuotient(amplitude_state_);
//   nn_input_vector_
//       << (target_pos - average_target_).cwiseQuotient(amplitude_target);

//   for (const auto& index : observed_state_ids_) {
//     nn_input_vector_
//         << (X[index] - average_state_).cwiseQuotient(amplitude_state_);
//   }
// }

const Eigen::VectorXd& DeburringONNXInterface::getReferenceState() {
  xref_ << x0_;
  for (size_t i = 0; i < rl_controlled_ids_.size(); ++i) {
    xref_[static_cast<Eigen::Index>(rl_controlled_ids_[i])] +=
        action_scale_ * nn_output_.data[i] *
        action_amplitude_[static_cast<Eigen::Index>(i)];
  }

  return (xref_);
}
