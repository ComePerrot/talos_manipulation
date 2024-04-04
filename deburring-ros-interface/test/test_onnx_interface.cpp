#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_ALTERNATIVE_INIT_API

#include <gtest/gtest.h>

#include "deburring_ros_interface/onnx_interface.h"

// Define a test fixture for the ONNXInterface class
class ONNXInterfaceTest : public ::testing::Test {
 protected:
  ONNXInterfaceTest() : onnx_interface_(nh_){};
  ros::NodeHandle nh_;
  DeburringONNXInterface onnx_interface_;

  // Tear down the test fixture
  virtual void TearDown() {
    // Shut down the ROS node
    ros::shutdown();
  }
};

// Define a test case for the ONNXInterface class
TEST_F(ONNXInterfaceTest, UpdateTest) {
  // Create some dummy input data
  int state_size = 57;
  int horizon_size = 101;
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(state_size);

  std::vector<Eigen::VectorXd> X;
  for (int i = 0; i < horizon_size; i++) {
    X.push_back(Eigen::VectorXd::Zero(state_size));
  }

  Eigen::Vector3d target_pos(1.0, 2.0, 3.0);

  // Call the update method with the dummy input data
  onnx_interface_.update(x0, X, target_pos);

  // Wait for the scaled_action message to be received
  ros::spinOnce();
  ros::Duration(0.1).sleep();

  auto scaled_action_msg_ = onnx_interface_.get_nnOutput();
  // Check that the scaled_action message was received
  EXPECT_TRUE(!scaled_action_msg_.data.empty());

  // Check that the scaled_action message has the correct size
  EXPECT_EQ(scaled_action_msg_.data.size(), 3);
}

// Define a test suite for the ONNXInterface class
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // Initialize the ROS node
  ros::init(argc, argv, "test_onnx_interface");

  // Run the test suite
  return RUN_ALL_TESTS();
}
