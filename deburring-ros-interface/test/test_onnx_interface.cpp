#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_ALTERNATIVE_INIT_API

#include <gtest/gtest.h>

#include "deburring_ros_interface/onnx_interface.h"

// Define a test fixture for the ONNXInterface class
class ONNXInterfaceTest : public ::testing::Test {
 protected:
  ONNXInterfaceTest() : onnx_interface_(nh_, 57, 101){};
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
  Eigen::VectorXd x0 = Eigen::VectorXd::Constant(state_size, 1.0);

  std::vector<Eigen::VectorXd> X;
  for (int i = 0; i < horizon_size; i++) {
    X.push_back(Eigen::VectorXd::Constant(state_size, i));
  }

  Eigen::Vector3d target_pos(2.0, 2.0, 2.0);

  // Call the update method with the dummy input data
  onnx_interface_.update(x0, X, target_pos);

  // Wait for the scaled_action message to be received
  ros::Rate rate(10);  // 10 Hz
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(0.5);  // 0.5 second timeout
  while (onnx_interface_.get_nnOutput().data.empty() && ros::ok()) {
    if (ros::Time::now() - start_time > timeout) {
      FAIL() << "Timed out waiting for scaled_action message";
    }
    ros::spinOnce();
    rate.sleep();
  }

  auto scaled_action_msg = onnx_interface_.get_nnOutput();
  // Check that the scaled_action message was received
  EXPECT_TRUE(!scaled_action_msg.data.empty());

  // Check that the scaled_action message has the correct size
  EXPECT_EQ(scaled_action_msg.data.size(), 3);

  auto xref = onnx_interface_.getReferenceState();

  // Check that the scaled_action message has the correct size
  EXPECT_EQ(xref.size(), state_size);
}

// Define a test suite for the ONNXInterface class
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // Initialize the ROS node
  ros::init(argc, argv, "test_onnx_interface");

  // Run the test suite
  return RUN_ALL_TESTS();
}
