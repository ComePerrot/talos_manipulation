#include "deburring_mpc/ocp.hpp"

#define PI 3.14159265

int main() {
  // Create OCP from configuration files
  //  Robot designer
  deburring::RobotDesignerSettings designerSettings =
      deburring::RobotDesignerSettings();

  designerSettings.controlled_joints_names = {
      "root_joint",        "leg_left_1_joint",  "leg_left_2_joint",
      "leg_left_3_joint",  "leg_left_4_joint",  "leg_left_5_joint",
      "leg_left_6_joint",  "leg_right_1_joint", "leg_right_2_joint",
      "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint",
      "leg_right_6_joint", "torso_1_joint",     "torso_2_joint",
      "arm_left_1_joint",  "arm_left_2_joint",  "arm_left_3_joint",
      "arm_left_4_joint",  "arm_left_5_joint",  "arm_left_6_joint",
      "arm_left_7_joint",  "arm_right_1_joint", "arm_right_2_joint",
      "arm_right_3_joint", "arm_right_4_joint"};
  designerSettings.left_foot_name = "left_sole_link";
  designerSettings.right_foot_name = "right_sole_link";
  designerSettings.urdf_path =
      "/opt/openrobots/share/example-robot-data/robots/talos_data/robots/"
      "talos_reduced.urdf";
  designerSettings.srdf_path =
      "/opt/openrobots/share/example-robot-data/robots/talos_data/srdf/"
      "talos.srdf";

  deburring::RobotDesigner pinWrapper = deburring::RobotDesigner(designerSettings);

  pinocchio::SE3 gripperMtool = pinocchio::SE3::Identity();
  gripperMtool.translation().x() = 0;
  gripperMtool.translation().y() = -0.02;
  gripperMtool.translation().z() = -0.0825;
  pinWrapper.addEndEffectorFrame("deburring_tool",
                                 "gripper_left_fingertip_3_link", gripperMtool);

  std::vector<double> lowerPositionLimit = {
      // Base
      -1, -1, -1, -1, -1, -1, -1,
      // Left leg
      -0.35, -0.52, -2.10, 0.0, -1.31, -0.52,
      // Right leg
      -1.57, -0.52, -2.10, 0.0, -1.31, -0.52,
      // Torso
      -1.3, -0.1,
      // Left arm
      -1.57, 0.2, -2.44, -2.1, -2.53, -1.3, -0.6,
      // Right arm
      -0.4, -2.88, -2.44, -2};
  std::vector<double> upperPositionLimit = {// Base
                                            1, 1, 1, 1, 1, 1, 1,
                                            // Left leg
                                            1.57, 0.52, 0.7, 2.62, 0.77, 0.52,
                                            // Right leg
                                            0.35, 0.52, 0.7, 2.62, 0.77, 0.52,
                                            // Torso
                                            1.3, 0.78,
                                            // Left arm
                                            0.52, 2.88, 2.44, 0, 2.53, 1.3, 0.6,
                                            // Right arm
                                            1.57, -0.2, 2.44, 0};

  std::vector<double>::size_type size_limit = lowerPositionLimit.size();
  pinWrapper.updateModelLimits(
      Eigen::VectorXd::Map(lowerPositionLimit.data(), (Eigen::Index)size_limit),
      Eigen::VectorXd::Map(upperPositionLimit.data(),
                           (Eigen::Index)size_limit));

  //  OCP
  std::string parameterFileOCP =
      "/home/cperrot/workspaces/wbDeburring/src/talos-manipulation/config/"
      "settings_sobec.yaml";
  deburring::OCPSettings_Point ocpSettings = deburring::OCPSettings_Point();
  ocpSettings.readParamsFromYamlFile(parameterFileOCP);
  deburring::OCP_Point OCP = deburring::OCP_Point(ocpSettings, pinWrapper);

  // Load data from serialized file
  std::vector<deburring::OCP_debugData>::size_type testCase = 0;
  std::vector<deburring::OCP_debugData> debugData = OCP.fetchFromFile(
      "/home/cperrot/workspaces/archives/"
      "OCP_2.txt");

  // Initialize OCP with the same initial state

  pinocchio::SE3 oMtarget;
  oMtarget.translation().x() = 0.6;
  oMtarget.translation().y() = 0.4;
  oMtarget.translation().z() = 1.1;

  // To have the gripper_left_fingertip_3_link in the right orientation we
  // need :
  //   90° around the y axis
  //   180° around the z axis
  double beta = -PI * 0.5;
  double gamma = PI;

  Eigen::Matrix3d rotationY;
  rotationY.row(0) << cos(beta), 0, -sin(beta);
  rotationY.row(1) << 0, 1, 0;
  rotationY.row(2) << sin(beta), 0, cos(beta);
  Eigen::Matrix3d rotationZ;
  rotationZ.row(0) << cos(gamma), sin(gamma), 0;
  rotationZ.row(1) << -sin(gamma), cos(gamma), 0;
  rotationZ.row(2) << 0, 0, 1;

  oMtarget.rotation() = rotationZ * rotationY;

  Eigen::VectorXd x_current = debugData[testCase].xi[0];

  OCP.initialize(x_current, oMtarget);
  OCP.reprOCP(0);

  // Compare loaded data with the one generated by the OCP
  auto solver = OCP.get_solver();
  auto data = debugData[testCase];

  // State
  auto xs_OCP = solver->get_xs();
  auto xs_Data = data.xs;

  // Command
  auto us_OCP = solver->get_us();
  auto us_Data = data.us;

  // Riccati
  auto K_OCP = solver->get_K();
  auto K_Data = data.K;

  for (std::vector<Eigen::VectorXd>::size_type i = 0; i < xs_OCP.size(); i++) {
    auto error = xs_OCP[i] - xs_Data[i];
    auto error_norm = error.norm();
    if (error_norm > 1e-10) {
      std::cout << "xs[" << i << "]"
                << "=";
      std::cout << error_norm << std::endl;
      std::cout << "Failure" << std::endl;
      return 1;
    }
  }

  for (std::vector<Eigen::VectorXd>::size_type i = 0; i < us_OCP.size(); i++) {
    auto error = us_OCP[i] - us_Data[i];
    auto error_norm = error.norm();
    if (error_norm > 1e-10) {
      std::cout << "us[" << i << "]"
                << "=";
      std::cout << error_norm << std::endl;
      std::cout << "Failure" << std::endl;
      return 1;
    }
  }

  for (std::vector<Eigen::VectorXd>::size_type i = 0; i < K_OCP.size(); i++) {
    auto error = K_OCP[i] - K_Data[i];
    auto error_norm = error.norm();
    if (error_norm > 1e-8) {
      std::cout << "K[" << i << "]"
                << "=";
      std::cout << error_norm << std::endl;
      std::cout << "Failure" << std::endl;
      return 1;
    }
  }

  std::cout << "Success" << std::endl;

  return 0;
}
