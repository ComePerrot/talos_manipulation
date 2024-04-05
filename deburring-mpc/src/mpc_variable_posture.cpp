#include "deburring_mpc/mpc.hpp"

namespace deburring {

void MPC::updateOCP_variablePosture(const VectorXd &x_ref) {
  updatePostureReference(x_ref);
  OCP_.recede();
  // Update posture
  OCP_.changePostureReference(OCP_.get_horizon_length() - 1,
                              updated_posture_ref_);
  OCP_.changePostureReference(OCP_.get_horizon_length(), updated_posture_ref_);

  // Last running node
  OCP_.changeGoalCostActivation(OCP_.get_horizon_length() - 1, true);
  // Terminal Node
  OCP_.changeGoalCostActivation(OCP_.get_horizon_length(), true);

  switch (drilling_state_) {
    case Status::kInitialization:  // Initialization
      if (iteration_ < settings_.T_initialization) {
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_ = Status::kFirstHole;
      }
      break;

    case Status::kFirstHole:  // Approaching target
      if (iteration_ == 0) {
        std::cout << "Initiating movement" << std::endl;
      }
      if (iteration_ <= OCP_.get_horizon_length() - 1) {
        OCP_.changeGoalCostActivation(OCP_.get_horizon_length() - 1, true);
        // Terminal Node
        OCP_.changeGoalCostActivation(OCP_.get_horizon_length(), true);
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_ = Status::kStabilization;
      }
      break;

    case Status::kStabilization:  // Waiting for arm to stabilize (and closing
                                  // loop on MOCAP feedback)
      if (iteration_ == 0) {
        std::cout << "Waiting for arm to stabilize" << std::endl;
      }
      if (iteration_ < settings_.T_stabilization) {
        iteration_++;
      } else {
        iteration_ = 0;
        if (position_error_ > 0.05) {
          drilling_state_ = Status::kHoleTransition;
        } else {
          drilling_state_ = Status::kDrilling;
        }
      }
      break;

    case Status::kDrilling:  // Drilling
      if (iteration_ == 0) {
        std::cout << "Initiating drilling phase" << std::endl;
      }
      if (iteration_ <= settings_.T_drilling) {
        iteration_++;
      } else {
        std::cout << "Drilling over" << std::endl;

        iteration_ = 0;
        drilling_state_ = Status::kDisengagement;
      }
      break;

    case Status::kDisengagement:  // Getting out of hole
      if (iteration_ == 0) {
        std::cout << "Getting out of the hole" << std::endl;
        oMdisengaged_target_ =
            list_oMhole_[current_hole_].act(backward_offset_);
      }
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changeTarget(OCP_.get_horizon_length() - iteration_,
                          oMdisengaged_target_.translation());
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_ = Status::kHoleTransition;
      }
      break;

    case Status::kHoleTransition:  // Switching to next target
      if (iteration_ == 0) {
        if (current_hole_ < (number_holes_ - 1)) {
          current_hole_++;
          std::cout << "Switching to target: " << current_hole_ + 1
                    << std::endl;
          iteration_++;
        } else {
          std::cout << "Going back to inital position" << std::endl;
          drilling_state_ = Status::kReturnHome;
        }
      }
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changeTarget(OCP_.get_horizon_length() - 1,
                          list_oMhole_[current_hole_].translation());
        OCP_.changeTarget(OCP_.get_horizon_length(),
                          list_oMhole_[current_hole_].translation());
        iteration_++;
      } else {
        iteration_ = 0;
        drilling_state_ = Status::kStabilization;
      }
      break;

    case Status::kReturnHome:  // Returning to initial position
      if (iteration_ <= OCP_.get_horizon_length()) {
        OCP_.changeGoalCostActivation(OCP_.get_horizon_length() - iteration_,
                                      false);
        iteration_++;
      } else {
        std::cout << "Robot is back to initial position" << std::endl;

        iteration_ = 0;
        drilling_state_ = Status::kDeburringDone;
      }
      break;

    case Status::kDeburringDone:  // Movement done
      break;
  }
}
}  // namespace deburring