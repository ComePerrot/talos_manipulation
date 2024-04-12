#include "deburring_mpc/mpc.hpp"

namespace deburring {

#define PI 3.14159265

MPC::MPC(const MPCSettings &mpc_settings, const OCPSettings &ocp_settings,
         const RobotDesigner &designer)
    : settings_(mpc_settings),
      designer_(designer),
      OCP_(ocp_settings, designer_) {
  backward_offset_.translation().z() = settings_.backward_offset;
  goal_weight_ = OCP_.get_settings().w_gripper_pos;
  for (auto offset : settings_.holes_offsets) {
    holes_offsets_.push_back(SE3(Matrix3d::Identity(), offset));
  }
}

void MPC::initialize(const ConstVectorRef &q0, const ConstVectorRef &v0,
                     const SE3 &toolMtarget) {
  controlled_joints_ids_ = designer_.get_controlled_joints_ids();
  x_internal_.resize(designer_.get_rmodel().nq + designer_.get_rmodel().nv);

  x0_.resize(designer_.get_rmodel().nq + designer_.get_rmodel().nv);
  x0_ << shapeState(q0, v0);
  designer_.updateReducedModel(x0_);
  // designer_.updateCompleteModel(q0);

  // Setup target
  setTarget(toolMtarget);
  setPostureReference(x0_);

  // Init OCP
  OCP_.initialize(x0_, oMtarget_);
  u0_ = OCP_.get_torque();
  K0_ = OCP_.get_gain();

  initialized_ = true;
}

// void MPC::iterate(const ConstVectorRef &q_current,
//                   const ConstVectorRef &v_current, const SE3 &toolMtarget) {
//   iterate(shapeState(q_current, v_current), toolMtarget);
// }

void MPC::iterate(const VectorXd &x0, const SE3 &toolMtarget) {
  x0_ = x0;

  designer_.updateReducedModel(x0_);

  updateTarget(toolMtarget);
  if (settings_.precision_strategy == 3) {
    updateOCP_variablePosture(x0_);
  } else {
    updateOCP();
  }

  OCP_.solve(x0_);

  u0_ = OCP_.get_torque();
  K0_ = OCP_.get_gain();
}

void MPC::iterate(const VectorXd &x0, const VectorXd &xref, const SE3 &toolMtarget) {
  x0_ = x0;

  designer_.updateReducedModel(x0_);

  updateTarget(toolMtarget);
  if (settings_.precision_strategy == 3) {
    updateOCP_variablePosture(xref);
  } else {
    updateOCP();
  }

  OCP_.solve(x0_);

  u0_ = OCP_.get_torque();
  K0_ = OCP_.get_gain();
}

void MPC::setTarget(const SE3 &toolMtarget) {
  // Setup target
  number_holes_ = settings_.holes_offsets.size();

  //  Define oMtarget
  if (settings_.use_mocap == 1 || settings_.use_mocap == 2) {
    toolMhole_ = toolMtarget.act(holes_offsets_[current_hole_]);
    oMtarget_ = designer_.get_end_effector_frame().act(toolMhole_);
  } else {
    oMtarget_.translation() = settings_.target_position;

    // To have the gripper_left_fingertip_3_link in the right orientation we
    // need :
    //   90° around the y axis
    //   180° around the z axis
    double beta = -PI * 0.5;
    double gamma = PI;

    Matrix3d rotationY;
    rotationY.row(0) << cos(beta), 0, -sin(beta);
    rotationY.row(1) << 0, 1, 0;
    rotationY.row(2) << sin(beta), 0, cos(beta);
    Matrix3d rotationZ;
    rotationZ.row(0) << cos(gamma), sin(gamma), 0;
    rotationZ.row(1) << -sin(gamma), cos(gamma), 0;
    rotationZ.row(2) << 0, 0, 1;

    oMtarget_.rotation() = rotationZ * rotationY;
  }

  //  Setup list_oMholes
  setHolesPlacement();

  oMtarget_hole_ = list_oMhole_[current_hole_];
}

void MPC::setHolesPlacement() {
  if (list_oMhole_.empty()) {
    for (size_t h = 0; h < number_holes_; h++) {
      list_oMhole_.push_back(oMtarget_.act(holes_offsets_[h]));
    }
  } else {
    for (size_t h = 0; h < number_holes_; h++) {
      list_oMhole_[h] = oMtarget_.act(holes_offsets_[h]);
    }
  }
}

void MPC::updateTarget(const SE3 &toolMtarget) {
  if (settings_.use_mocap == 0 || settings_.use_mocap == 1) {
    toolMhole_ =
        designer_.get_end_effector_frame().actInv(list_oMhole_[current_hole_]);
    position_error_ = toolMhole_.translation().norm();
  } else if (settings_.use_mocap == 2) {
    toolMhole_ = toolMtarget.act(holes_offsets_[current_hole_]);

    position_error_ = toolMhole_.translation().norm();

    if (position_error_ < 0.05 && drilling_state_ == Status::kStabilization) {
      oMtarget_ = designer_.get_end_effector_frame().act(toolMhole_);

      setHolesPlacement();
      OCP_.updateGoalPosition(list_oMhole_[current_hole_].translation());
    }
  }
  oMtarget_hole_ = list_oMhole_[current_hole_];
}

void MPC::setPostureReference(const ConstVectorRef &x0) {
  initial_posture_ref_ = x0;
  initial_posture_ref_.tail(designer_.get_rmodel().nv) =
      VectorXd::Zero(designer_.get_rmodel().nv);

  updated_posture_ref_ = initial_posture_ref_;
  updated_posture_ref_.segment(21, 4) = settings_.custom_arm_ref;
}

void MPC::updatePostureReference(const ConstVectorRef &x) {
  updated_posture_ref_.segment(21, 4) = x.segment(21, 4);
}

const VectorXd &MPC::shapeState(const ConstVectorRef &q,
                                const ConstVectorRef &v) {
  if (q.size() == designer_.get_rmodel_complete().nq &&
      v.size() == designer_.get_rmodel_complete().nv) {
    x_internal_.head<7>() = q.head<7>();
    x_internal_.segment<6>(designer_.get_rmodel().nq) = v.head<6>();

    int i = 0;
    for (unsigned long jointID : controlled_joints_ids_)
      if (jointID > 1) {
        x_internal_(i + 7) = q((long)jointID + 5);
        x_internal_(designer_.get_rmodel().nq + i + 6) = v((long)jointID + 4);
        i++;
      }
    return x_internal_;
  } else if (q.size() == designer_.get_rmodel().nq &&
             v.size() == designer_.get_rmodel().nv) {
    x_internal_ << q, v;
    return x_internal_;
  } else
    throw std::runtime_error(
        "q and v must have the dimentions of the reduced or complete model.");
}
}  // namespace deburring
