#include "deburring_mpc/ocp.hpp"

namespace deburring {
void OCP::defineFeetContact(Contact &contact_collector) {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> contact_model_left =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_lf_id(), designer_.get_lf_frame(),
          actuation_->get_nu(), Eigen::Vector2d(0., 4.));

  boost::shared_ptr<crocoddyl::ContactModelAbstract> contact_model_right =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_rf_id(), designer_.get_rf_frame(),
          actuation_->get_nu(), Vector2d(0., 4.));

  contact_collector->addContact(designer_.get_lf_name(), contact_model_left,
                                false);
  contact_collector->addContact(designer_.get_rf_name(), contact_model_right,
                                false);

  contact_collector->changeContactStatus(designer_.get_lf_name(), true);
  contact_collector->changeContactStatus(designer_.get_rf_name(), true);
}

void OCP::defineFeetWrenchCost(CostModelSum &costCollector,
                               const double w_wrench) {
  crocoddyl::WrenchCone wrenchCone_LF =
      crocoddyl::WrenchCone(Eigen::Matrix3d::Identity(), 0.3,
                            Eigen::Vector2d(0.1, 0.05), 4, true, 200.0, 1200);
  crocoddyl::WrenchCone wrenchCone_RF =
      crocoddyl::WrenchCone(Eigen::Matrix3d::Identity(), 0.3,
                            Eigen::Vector2d(0.1, 0.05), 4, true, 200.0, 1200);

  boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone>
      residual_LF_Wrench =
          boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(
              state_, designer_.get_lf_id(), wrenchCone_LF,
              actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_,
                                                       residual_LF_Wrench);

  boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone>
      residual_RF_Wrench =
          boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(
              state_, designer_.get_rf_id(), wrenchCone_RF,
              actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_,
                                                       residual_RF_Wrench);

  costCollector.get()->addCost("wrench_LF", wrenchModel_LF, w_wrench, true);
  costCollector.get()->addCost("wrench_RF", wrenchModel_RF, w_wrench, true);
}

void OCP::defineStateRegularization(CostModelSum &cost_collector,
                                    const double w_state_reg) {
  if (settings_.state_weights.size() != designer_.get_rmodel().nv * 2) {
    throw std::invalid_argument("State weight size is wrong ");
  }
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad>
      activation_weighted_quad =
          boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
              settings_.state_weights);

  boost::shared_ptr<crocoddyl::CostModelAbstract> state_regularization_cost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activation_weighted_quad,
          boost::make_shared<crocoddyl::ResidualModelState>(
              state_, designer_.get_x0(), actuation_->get_nu()));

  cost_collector.get()->addCost("stateRegularization",
                                state_regularization_cost, w_state_reg, true);
}

void OCP::defineControlRegularization(CostModelSum &cost_collector,
                                      const double w_control_reg) {
  if (settings_.control_weights.size() != (int)actuation_->get_nu()) {
    throw std::invalid_argument("Control weight size is wrong ");
  }
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad>
      activation_weighted_quad =
          boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
              settings_.control_weights);  //.tail(actuation->get_nu())

  boost::shared_ptr<crocoddyl::CostModelAbstract> control_regularization_cost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activation_weighted_quad,
          boost::make_shared<crocoddyl::ResidualModelControl>(
              state_, actuation_->get_nu()));
  cost_collector.get()->addCost("controlRegularization",
                                control_regularization_cost, w_control_reg,
                                true);
}

void OCP::defineStateLimits(CostModelSum &cost_collector,
                            const double w_state_limits,
                            const double limit_scale, const bool limit_speed) {
  Eigen::VectorXd lower_bound(2 * state_->get_nv()),
      upper_bound(2 * state_->get_nv());
  double inf = 9999.0;

  if (limit_speed) {
    lower_bound << Eigen::VectorXd::Constant(6, -inf),
        designer_.get_rmodel().lowerPositionLimit.tail(
            static_cast<Eigen::Index>(state_->get_nq() - 7)),
        Eigen::VectorXd::Constant(6, -inf),
        -designer_.get_rmodel().velocityLimit.tail(
            static_cast<Eigen::Index>(state_->get_nv() - 6));

    upper_bound << Eigen::VectorXd::Constant(6, inf),
        designer_.get_rmodel().upperPositionLimit.tail(
            static_cast<Eigen::Index>(state_->get_nq() - 7)),
        Eigen::VectorXd::Constant(6, inf),
        designer_.get_rmodel().velocityLimit.tail(
            static_cast<Eigen::Index>(state_->get_nv() - 6));
  } else {
    lower_bound << Eigen::VectorXd::Constant(6, -inf),
        designer_.get_rmodel().lowerPositionLimit.tail(
            static_cast<Eigen::Index>(state_->get_nq() - 7)),
        Eigen::VectorXd::Constant(static_cast<Eigen::Index>(state_->get_nv()),
                                  -inf);

    upper_bound << Eigen::VectorXd::Constant(6, inf),
        designer_.get_rmodel().upperPositionLimit.tail(
            static_cast<Eigen::Index>(state_->get_nq() - 7)),
        Eigen::VectorXd::Constant(static_cast<Eigen::Index>(state_->get_nv()),
                                  inf);
  }

  crocoddyl::ActivationBounds bounds =
      crocoddyl::ActivationBounds(lower_bound, upper_bound, limit_scale);

  boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier>
      activation_quad_barrier =
          boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(
              bounds);
  boost::shared_ptr<crocoddyl::CostModelAbstract> joint_limit_cost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activation_quad_barrier,
          boost::make_shared<crocoddyl::ResidualModelState>(
              state_, actuation_->get_nu()));

  cost_collector.get()->addCost("jointLimits", joint_limit_cost, w_state_limits,
                                true);
}

void OCP::defineControlLimits(CostModelSum &cost_collector,
                              const double w_control_limit,
                              const double limit_scale) {
  Eigen::VectorXd lower_bound(actuation_->get_nu()),
      upper_bound(actuation_->get_nu());

  lower_bound << -designer_.get_rmodel().effortLimit.tail(
      static_cast<Eigen::Index>(actuation_->get_nu()));
  upper_bound << designer_.get_rmodel().effortLimit.tail(
      static_cast<Eigen::Index>(actuation_->get_nu()));

  crocoddyl::ActivationBounds bounds =
      crocoddyl::ActivationBounds(lower_bound, upper_bound, limit_scale);

  boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier>
      activation_quad_barrier =
          boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(
              bounds);
  boost::shared_ptr<crocoddyl::CostModelAbstract> control_limit_cost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activation_quad_barrier,
          boost::make_shared<crocoddyl::ResidualModelControl>(
              state_, actuation_->get_nu()));

  cost_collector.get()->addCost("controlLimits", control_limit_cost,
                                w_control_limit, true);
}

void OCP::defineCoMPosition(CostModelSum &cost_collector,
                            const double w_com_pos) {
  Vector3d com_ref_position = designer_.get_com_position();
  boost::shared_ptr<crocoddyl::CostModelAbstract> com_posistion_cost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelCoMPosition>(
                      state_, com_ref_position, actuation_->get_nu()));

  cost_collector.get()->addCost("comPosition", com_posistion_cost, w_com_pos,
                                true);
}

void OCP::defineGripperPlacement(CostModelSum &cost_collector,
                                 const double w_gripper_pos,
                                 const double w_gripper_rot) {
  pinocchio::SE3 goal_placement = pinocchio::SE3::Identity();

  // Position
  boost::shared_ptr<crocoddyl::CostModelAbstract> gripper_position_cost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_,
          boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(3, 0.02),
          boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
              state_, designer_.get_end_effector_id(),
              goal_placement.translation(), actuation_->get_nu()));

  cost_collector.get()->addCost("gripperPosition", gripper_position_cost,
                                w_gripper_pos, true);

  // Orientation
  boost::shared_ptr<crocoddyl::CostModelAbstract> gripper_rotation_cost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
                      state_, designer_.get_end_effector_id(),
                      goal_placement.rotation(), actuation_->get_nu()));

  cost_collector.get()->addCost("gripperRotation", gripper_rotation_cost,
                                w_gripper_rot, true);
}

void OCP::defineGripperVelocity(CostModelSum &cost_collector,
                                const double w_gripper_vel) {
  pinocchio::Motion goal_motion = pinocchio::Motion(Eigen::VectorXd::Zero(6));
  boost::shared_ptr<crocoddyl::CostModelAbstract> gripper_velocity_cost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(
                      state_, designer_.get_end_effector_id(), goal_motion,
                      pinocchio::WORLD, actuation_->get_nu()));

  cost_collector.get()->addCost("gripperVelocity", gripper_velocity_cost,
                                w_gripper_vel, true);
}

}  // namespace deburring