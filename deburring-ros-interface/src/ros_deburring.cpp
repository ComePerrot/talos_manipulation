#include <deburring_mpc/mpc.hpp>
// Must be included first

#include <pal_statistics/pal_statistics_macros.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include "deburring_ros_interface/custom_registration_utils.h"
#include "deburring_ros_interface/onnx_interface.h"
#include "deburring_ros_interface/ros_interface.h"

deburring::RobotDesigner buildRobotDesigner(ros::NodeHandle nh) {
  // Settings
  deburring::RobotDesignerSettings designer_settings =
      deburring::RobotDesignerSettings();
  nh.getParam("left_foot_name", designer_settings.left_foot_name);
  nh.getParam("right_foot_name", designer_settings.right_foot_name);
  nh.getParam("urdf", designer_settings.urdf_path);
  nh.getParam("srdf", designer_settings.srdf_path);
  nh.getParam("robot_description", designer_settings.robot_description);
  nh.getParam("controlled_joints", designer_settings.controlled_joints_names);

  std::vector<double> gripperTtool;
  nh.getParam("tool_frame_pos", gripperTtool);
  pinocchio::SE3 gripperMtool = pinocchio::SE3::Identity();
  gripperMtool.translation().x() = gripperTtool[0];
  gripperMtool.translation().y() = gripperTtool[1];
  gripperMtool.translation().z() = gripperTtool[2];

  ROS_INFO_STREAM(designer_settings.urdf_path);

  ROS_INFO_STREAM("Building robot designer");
  deburring::RobotDesigner designer =
      deburring::RobotDesigner(designer_settings);

  ROS_INFO_STREAM("Adding end effector frame to the robot model");
  designer.addEndEffectorFrame("deburring_tool",
                               "gripper_left_fingertip_3_link", gripperMtool);

  bool use_custom_limits;
  nh.getParam("custom_limits", use_custom_limits);

  if (use_custom_limits) {
    ROS_INFO_STREAM("Updating Limits");
    // Loading custom model limits
    std::vector<double> lower_position_limits;
    nh.getParam("lowerPositionLimit", lower_position_limits);
    std::vector<double> upper_position_limits;
    nh.getParam("upperPositionLimit", upper_position_limits);

    std::vector<double>::size_type size_limit = lower_position_limits.size();

    designer.updateModelLimits(
        Eigen::VectorXd::Map(lower_position_limits.data(),
                             (Eigen::Index)size_limit),
        Eigen::VectorXd::Map(upper_position_limits.data(),
                             (Eigen::Index)size_limit));
  }

  return (designer);
}

deburring::MPC buildMPC(ros::NodeHandle nh,
                        const deburring::RobotDesigner& pinWrapper) {
  std::string parameterFileName;
  nh.getParam("controller_settings_file", parameterFileName);
  std::string parameterFilePath =
      ros::package::getPath("deburring_ros_interface") + "/config/";
  std::string parameterFile = parameterFilePath + parameterFileName;

  deburring::OCPSettings ocpSettings = deburring::OCPSettings();
  deburring::MPCSettings mpcSettings = deburring::MPCSettings();

  ocpSettings.readParamsFromYamlFile(parameterFile);
  mpcSettings.readParamsFromYamlFile(parameterFile);

  deburring::MPC mpc = deburring::MPC(mpcSettings, ocpSettings, pinWrapper);

  return (mpc);
}

class MoCapInterface {
 public:
  MoCapInterface(int use_mocap) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (use_mocap) {
      while (toolMtarget_.isIdentity()) {
        readTF();
        ROS_INFO_THROTTLE(0.5, "Receiving target position from the MOCAP");
        ros::spinOnce();
      }
    }
  }

  pinocchio::SE3& get_toolMtarget() {
    readTF();
    return (toolMtarget_);
  }

 private:
  void readTF() {
    try {
      transform_stamped_ =
          tf_buffer_->lookupTransform("target", "tool", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
    }
    tf::transformMsgToEigen(transform_stamped_.transform, eigen_transform_);
    toolMtarget_ = pinocchio::SE3(eigen_transform_.rotation(),
                                  eigen_transform_.translation());
  }

  // TF variables
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Target position wrt tool
  pinocchio::SE3 toolMtarget_ = pinocchio::SE3::Identity();

  // Memmory allocation
  geometry_msgs::TransformStamped transform_stamped_;
  Eigen::Affine3d eigen_transform_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_mpc_pointing");
  ros::NodeHandle nh;
  pal_statistics::RegistrationsRAII registered_variables;

  // Timing variables
  ros::Time OCP_start_time = ros::Time::now();
  ros::Time OCP_end_time = ros::Time::now();
  double OCP_solve_time;

  // Robot Desginer & MPC
  deburring::RobotDesigner pin_wrapper = buildRobotDesigner(nh);
  deburring::MPC MPC = buildMPC(nh, pin_wrapper);

  // Mocap Interface
  int use_mocap = MPC.get_settings().use_mocap;
  MoCapInterface Mocap = MoCapInterface(use_mocap);

  // Robot Interface
  DeburringROSInterface Robot = DeburringROSInterface(nh);

  // Neural Network interface
  bool deep_planner;
  nh.getParam("ros_deburring/deep_planner", deep_planner);
  DeburringONNXInterface NeuralNet = DeburringONNXInterface(
      nh, static_cast<size_t>(pin_wrapper.get_x0().size()),
      MPC.get_OCP().get_horizon_length(),
      deep_planner);

  // Initialize MPC
  pinocchio::SE3 toolMtarget;

  if (use_mocap > 0) {
    toolMtarget = Mocap.get_toolMtarget();
  } else {
    toolMtarget = pinocchio::SE3::Identity();
  }

  Eigen::VectorXd x_measured = Robot.get_robotState();
  MPC.initialize(x_measured.head(MPC.get_designer().get_rmodel().nq),
                 x_measured.tail(MPC.get_designer().get_rmodel().nv),
                 toolMtarget);
  if (deep_planner) {
    NeuralNet.update(x_measured, MPC.get_OCP().get_solver()->get_xs(),
                     MPC.get_target_frame().translation());
  }

  REGISTER_VARIABLE("/introspection_data", "end_effector_actual_position",
                    &MPC.get_designer().get_end_effector_frame().translation(),
                    &registered_variables);
  REGISTER_VARIABLE("/introspection_data", "end_effector_desired_position",
                    &MPC.get_target_frame().translation(),
                    &registered_variables);
  REGISTER_VARIABLE("/introspection_data", "end_effector_position_error",
                    &MPC.get_position_error(), &registered_variables);
  REGISTER_VARIABLE("/introspection_data", "end_effector_position_task_weight",
                    &MPC.get_goal_weight(), &registered_variables);
  REGISTER_VARIABLE("/introspection_data", "OCP_solve_time", &OCP_solve_time,
                    &registered_variables);

  Eigen::VectorXd x_ref;
  Eigen::VectorXd u0;
  Eigen::MatrixXd K0;
  Eigen::VectorXd croco_contact_left_;
  croco_contact_left_.resize(6);
  Eigen::VectorXd croco_contact_right_;
  croco_contact_right_.resize(6);

  ros::Rate r(static_cast<double>(1 / MPC.get_OCP().get_settings().time_step));
  while (ros::ok()) {
    ros::spinOnce();

    // Get state from Robot intergace
    x_measured = Robot.get_robotState();
    if (deep_planner) {
      NeuralNet.update(x_measured, MPC.get_OCP().get_solver()->get_xs(),
                       MPC.get_target_frame().translation());
      x_ref = NeuralNet.getReferenceState();
    } else {
      x_ref = x_measured;
    }

    // Solving MPC iteration
    OCP_start_time = ros::Time::now();
    MPC.iterate(x_measured, x_ref, toolMtarget);
    OCP_end_time = ros::Time::now();
    OCP_solve_time = (OCP_end_time - OCP_start_time).toSec();

    // Sending command to robot
    u0 = MPC.get_u0();
    K0 = MPC.get_K0();
    croco_contact_left_ << MPC.get_OCP().getFootForce(0, "wrench_LF"),
        MPC.get_OCP().getFootTorque(0, "wrench_LF");
    croco_contact_right_ << MPC.get_OCP().getFootForce(0, "wrench_RF"),
        MPC.get_OCP().getFootTorque(0, "wrench_RF");

    Robot.update(u0, K0, croco_contact_left_, croco_contact_right_);

    PUBLISH_STATISTICS("/introspection_data");

    r.sleep();
  }

  return (0);
}
