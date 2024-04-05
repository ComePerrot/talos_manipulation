///////////////////////////////////////////////////////////////////////////////
// BSD 2-Clause License
//
// Copyright (C) 2022-2023, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef DEBURRING_MPC
#define DEBURRING_MPC

#include "deburring_mpc/fwd.hpp"
#include "deburring_mpc/ocp.hpp"
#include "deburring_mpc/robot_designer.hpp"

namespace deburring {

struct MPC_command {
 public:
  Eigen::VectorXd us0;
  Eigen::MatrixXd K0;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /*version*/) {
    ar &us0;
    ar &K0;
  }
};

struct MPC_debugData {
 public:
  Eigen::VectorXd x_input;

  MPC_command output;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /*version*/) {
    ar &x_input;

    ar &output;
  }
};

struct MPCSettings {
  // Timing parameters
  size_t T_initialization;
  size_t T_stabilization;
  size_t T_drilling;

  // Target parameters
  Vector3d target_position;
  std::vector<Vector3d> holes_offsets;
  double backward_offset;
  double precision_threshold;
  int precision_strategy;

  // Gain Scheduling
  // int use_gain_scheduling;
  double gain_schedulig_slope;
  double gain_schedulig_max_weight;

  // Variable Posture
  VectorXd custom_arm_ref;

  // Mocap
  int use_mocap;

  void readParamsFromYamlString(std::string &string_to_parse);
  void readParamsFromYamlFile(const std::string &filename);
};

/**
 * @brief Status of the MPC
 */
enum class Status {
  kInitialization,
  kStabilization,
  kDrilling,
  kDisengagement,
  kFirstHole,
  kHoleTransition,
  kReturnHome,
  kIncreaseGain,
  kUpdatePosture,
  kDeburringDone
};

/**
 * @brief Model Predictive Controller
 *
 * This class handles the calls to the OCP.
 * A state machine is used to manage the transition between the variouses phases
 * of the movement
 */
class MPC {
 private:
  MPCSettings settings_;
  RobotDesigner designer_;
  OCP OCP_;

  VectorXd x0_;
  VectorXd u0_;
  MatrixXd K0_;

  // MPC State
  size_t current_hole_ = 0;
  Status drilling_state_ = Status::kInitialization;
  size_t iteration_ = 0;
  double goal_weight_ = 0;

  // Target related variables
  size_t number_holes_;
  std::vector<SE3> holes_offsets_;
  std::vector<SE3> list_oMhole_;  // Holes position in the robot frame
  SE3 backward_offset_ = SE3::Identity();

  VectorXd initial_posture_ref_;
  VectorXd updated_posture_ref_;

  // Security management
  bool initialized_ = false;

  // Memory preallocations:
  SE3 oMtarget_;
  SE3 oMdisengaged_target_;
  SE3 oMtarget_hole_;
  SE3 toolMhole_;
  double position_error_ = 0;
  std::vector<unsigned long> controlled_joints_ids_;
  VectorXd x_internal_;

 private:
  void setTarget(const SE3 &toolMtarget);
  void setHolesPlacement();
  void updateTarget(const SE3 &toolMtarget);
  void setPostureReference(const ConstVectorRef &x0);
  void updatePostureReference(const ConstVectorRef &x);
  void updateOCP();
  void updateOCP_variablePosture(const VectorXd &x_meas);

 public:
  /**
   * @brief Construct a new MPC object
   *
   * The object is created but the problem is not initialized yet.
   *
   * @param mpc_settings Settings which affect the behavior of the state machine
   * @param ocp_settings Settings necessary for the creation of the OCP
   * @param designer RobotDesigner object which handles the calls to pinocchio
   */
  MPC(const MPCSettings &mpc_settings, const OCPSettings &ocp_settings,
      const RobotDesigner &designer);

  /**
   * @brief Initialize the MPC
   *
   * Runs the OCP to convergence on to solve the first formulation of the
   * problem. This resolution is used as a warm-start for subsequent iterations
   * of the MPC.
   *
   * @param q0 Initial joint position (including state of the base)
   * @param v0 Initial joint velocity (including state of the base)
   * @param toolMtarget Position of the target wrt the tool frame
   */
  void initialize(const ConstVectorRef &q0, const ConstVectorRef &v0,
                  const SE3 &toolMtarget);

  /**
   * @brief Do one iteration of the OCP
   *
   * State of the robot is used to update pinocchio data.
   * The formulation of the OCP is updated.
   * The OCP is solved to provide a new control to the robot.
   *
   * @param x0 Initial state of the robot (position and velocity)
   * @param toolMtarget Position of the target wrt the tool frame
   */
  void iterate(const VectorXd &x0, const SE3 &toolMtarget);
  void iterate(const VectorXd &x0, const VectorXd &xref,
               const SE3 &toolMtarget);

  /**
   * @brief Do one iteration of the OCP
   *
   * @param q_current Joint position (including state of the base)
   * @param v_current Joint velocity (including state of the base)
   * @param toolMtarget Position of the target wrt the tool frame
   */
  void iterate(const ConstVectorRef &q_current, const ConstVectorRef &v_current,
               const SE3 &toolMtarget);

  /**
   * @brief Shape the state of the robot according to the OCP definition
   *
   * @param q Joint position (including state of the base)
   * @param v Joint velocity (including state of the base)
   * @return The shaped state vector
   */
  const VectorXd &shapeState(const ConstVectorRef &q, const ConstVectorRef &v);

  // Debug
  std::vector<MPC_debugData> debugDataMPC_;

  void logData(const Eigen::Ref<const Eigen::VectorXd> x_input,
               const Eigen::Ref<const Eigen::VectorXd> us0,
               const Eigen::Ref<const Eigen::MatrixXd> K0);

  void dumpToFile(std::string name);
  std::vector<MPC_debugData> fetchFromFile(std::string name);

  // getters and setters
  MPCSettings &get_settings() { return settings_; }

  int get_drilling_state() const { return static_cast<int>(drilling_state_); }

  const VectorXd &get_x0() const { return x0_; }

  const VectorXd &get_u0() const { return u0_; }

  const MatrixXd &get_K0() const { return K0_; }

  const SE3 &get_target_frame() const { return oMtarget_hole_; }

  const double &get_position_error() const { return position_error_; }

  const double &get_goal_weight() const { return goal_weight_; }

  OCP &get_OCP() { return OCP_; }
  void set_OCP(const OCP &OCP) { OCP_ = OCP; }

  RobotDesigner &get_designer() { return designer_; }
  void set_designer(const RobotDesigner &designer) { designer_ = designer; }
};
}  // namespace deburring

#endif  // DEBURRING_MPC
