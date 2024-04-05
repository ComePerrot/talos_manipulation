#include <pinocchio/multibody/fwd.hpp>
// Must be included first!

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <eigenpy/eigenpy.hpp>

#include "deburring_mpc/fwd.hpp"
#include "deburring_mpc/mpc.hpp"

namespace deburring {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

template <typename T>
inline void py_list_to_std_vector(const bp::object &iterable,
                                  std::vector<T> &out) {
  out = std::vector<T>(boost::python::stl_input_iterator<T>(iterable),
                       boost::python::stl_input_iterator<T>());
}

static boost::shared_ptr<MPC> constructor(bp::dict mpc_settigns,
                                          bp::dict ocp_settings,
                                          const RobotDesigner designer) {
  OCPSettings conf_ocp;
  conf_ocp.horizon_length = bp::extract<size_t>(ocp_settings["horizon_length"]);
  conf_ocp.time_step = bp::extract<double>(ocp_settings["time_step"]);

  conf_ocp.w_state_reg = bp::extract<double>(ocp_settings["w_state_reg"]);
  conf_ocp.w_control_reg = bp::extract<double>(ocp_settings["w_control_reg"]);
  conf_ocp.w_state_limits = bp::extract<double>(ocp_settings["w_state_limits"]);
  conf_ocp.w_control_limit =
      bp::extract<double>(ocp_settings["w_control_limit"]);
  conf_ocp.w_com_pos = bp::extract<double>(ocp_settings["w_com_pos"]);
  conf_ocp.w_gripper_pos = bp::extract<double>(ocp_settings["w_gripper_pos"]);
  conf_ocp.w_gripper_rot = bp::extract<double>(ocp_settings["w_gripper_rot"]);
  conf_ocp.w_gripper_vel = bp::extract<double>(ocp_settings["w_gripper_vel"]);

  conf_ocp.limit_scale = bp::extract<double>(ocp_settings["limit_scale"]);
  conf_ocp.limit_speed = bp::extract<bool>(ocp_settings["limit_speed"]);

  conf_ocp.state_weights =
      bp::extract<Eigen::VectorXd>(ocp_settings["state_weights"]);
  conf_ocp.control_weights =
      bp::extract<Eigen::VectorXd>(ocp_settings["control_weights"]);

  MPCSettings conf_mpc;
  conf_mpc.T_initialization =
      bp::extract<size_t>(mpc_settigns["T_initialization"]);
  conf_mpc.T_stabilization =
      bp::extract<size_t>(mpc_settigns["T_stabilization"]);
  conf_mpc.T_drilling = bp::extract<size_t>(mpc_settigns["T_drilling"]);
  conf_mpc.target_position =
      bp::extract<Vector3d>(mpc_settigns["target_position"]);
  py_list_to_std_vector(mpc_settigns["holes_offsets"], conf_mpc.holes_offsets);
  conf_mpc.backward_offset =
      bp::extract<double>(mpc_settigns["backward_offset"]);
  conf_mpc.precision_threshold =
      bp::extract<double>(mpc_settigns["precision_threshold"]);
  conf_mpc.precision_strategy =
      bp::extract<int>(mpc_settigns["precision_strategy"]);
  conf_mpc.use_mocap = bp::extract<int>(mpc_settigns["use_mocap"]);
  conf_mpc.gain_schedulig_slope =
      bp::extract<double>(mpc_settigns["gain_schedulig_slope"]);
  conf_mpc.gain_schedulig_max_weight =
      bp::extract<double>(mpc_settigns["gain_schedulig_max_weight"]);
  conf_mpc.custom_arm_ref =
      bp::extract<VectorXd>(mpc_settigns["custom_arm_ref"]);

  return (boost::shared_ptr<MPC>(new MPC(conf_mpc, conf_ocp, designer)));
}

void exposeMPCClass() {
  bp::register_ptr_to_python<boost::shared_ptr<MPC>>();

  bp::class_<MPC>("MPC", bp::no_init)
      .def("__init__", bp::make_constructor(&constructor))
      .def<void (MPC::*)(const ConstVectorRef &, const ConstVectorRef &,
                         const SE3 &)>(
          "initialize", &MPC::initialize,
          bp::args("self", "q0", "v0", "toolMtarget"))
    //   .def<void (MPC::*)(const ConstVectorRef &, const ConstVectorRef &,
    //                      const SE3 &)>(
    //       "iterate", &MPC::iterate,
    //       bp::args("self", "q_current", "v_current", "toolMtarget"))
      .def<void (MPC::*)(const VectorXd &, const SE3 &)>(
          "iterate", &MPC::iterate, bp::args("self", "x0", "toolMtarget"))
      .add_property(
          "designer",
          bp::make_function(
              &MPC::get_designer,
              bp::return_value_policy<bp::reference_existing_object>()),
          "pinocchio wrapper used by the MPC")
      .add_property(
          "OCP",
          bp::make_function(
              &MPC::get_OCP,
              bp::return_value_policy<bp::reference_existing_object>()),
          "crocoddyl wrapper used by the MPC")
      .add_property(
          "oMtarget",
          bp::make_function(
              &MPC::get_target_frame,
              bp::return_value_policy<bp::reference_existing_object>()),
          "placement of the target in the robot frame")
      .add_property(
          "x0",
          bp::make_function(
              &MPC::get_x0,
              bp::return_value_policy<bp::reference_existing_object>()),
          "initial State")
      .add_property(
          "u0",
          bp::make_function(
              &MPC::get_u0,
              bp::return_value_policy<bp::reference_existing_object>()),
          "torque")
      .add_property(
          "K0",
          bp::make_function(
              &MPC::get_K0,
              bp::return_value_policy<bp::reference_existing_object>()),
          "Riccati gains")
      .add_property("drilling_state", &MPC::get_drilling_state,
                    "Riccati gains");
}

void exposeMPC() { exposeMPCClass(); }

}  // namespace python
}  // namespace deburring
