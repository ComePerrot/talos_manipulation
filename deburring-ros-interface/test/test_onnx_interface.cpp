#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_ALTERNATIVE_INIT_API

#include <boost/function.hpp>
#include <boost/test/execution_monitor.hpp>
#include <boost/test/included/unit_test.hpp>

#include "deburring_ros_interface/onnx_interface.h"

using namespace boost::unit_test;

void testONNXInterface() {
  ros::NodeHandle nh;

  DeburringONNXInterface onnx_interface(nh);

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
  onnx_interface.update(x0, X, target_pos);

  ros::spin();
}

void registerMPCUnitTest() {
  test_suite* ts = BOOST_TEST_SUITE("test_MPC");
  ts->add(BOOST_TEST_CASE(testONNXInterface));
  framework::master_test_suite().add(ts);
}

bool init_function() {
  registerMPCUnitTest();
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "onnx_interface");
  return ::boost::unit_test::unit_test_main(&init_function, argc, argv);
}
