#include "deburring_ros_interface/onnx_interface.h"

DeburringONNXInterface::DeburringONNXInterface(ros::NodeHandle nh) {
  ros::TransportHints hints;
  hints.tcpNoDelay(true);

  // NN action subscriber
  nn_sub_ = nh.subscribe("/scaled_action", 1, &DeburringONNXInterface::nnCb,
                         this, hints);

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
  // Concatenate the input vectors into a single vector
  int num_vectors = X.size();
  int num_elements = x0.size() + num_vectors * X[0].size() + target_pos.size();
  Eigen::VectorXd input_vector(num_elements);
  input_vector << x0, X[0], X[1], X[2], target_pos;

  // Create a new message to send to the nn_input topic
  std_msgs::Float64MultiArray nn_input_msg;
  nn_input_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  nn_input_msg.layout.dim[0].size = num_elements;
  nn_input_msg.layout.dim[0].stride = num_elements;
  nn_input_msg.layout.dim[0].label = "input";
  nn_input_msg.data.resize(num_elements);

  // Copy the concatenated vector into the message data array
  for (int i = 0; i < num_elements; ++i) {
    nn_input_msg.data[i] = input_vector[i];
  }

  // Publish the nn_input_msg
  nn_pub_.publish(nn_input_msg);
}

