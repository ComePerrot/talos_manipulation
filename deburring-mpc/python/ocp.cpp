#include <pinocchio/multibody/fwd.hpp>
// #include <pinocchio/multibody/fwd.hpp>

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <eigenpy/eigenpy.hpp>

#include "deburring_mpc/fwd.hpp"
#include "deburring_mpc/ocp.hpp"

namespace mpc_p {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeOCPPointParams() {
  bp::register_ptr_to_python<boost::shared_ptr<OCPSettings_Point>>();

  bp::class_<OCPSettings_Point>(
      "OCPSettings_Point",
      bp::init<>(bp::args("self"), "Empty initialization of the OCP params"))
      .def("readFromYaml", &OCPSettings_Point::readParamsFromYamlFile,
           bp::args("filename"))
      .add_property("horizon_length",
                    bp::make_getter(&OCPSettings_Point::horizon_length),
                    bp::make_setter(&OCPSettings_Point::horizon_length),
                    "Number of nodes in the horizon");
}

void exposeOCPPointClass() {
  bp::register_ptr_to_python<boost::shared_ptr<OCP_Point>>();

  bp::class_<OCP_Point>(
      "OCP_Point",
      bp::init<const OCPSettings_Point &, const RobotDesigner &>(
          bp::args("self", "OCPSettings", "designer"),
          "Initialize the OCP from parameter list and a robot designer"))
      .def<void (OCP_Point::*)(const ConstVectorRef &, const SE3 &)>(
          "initialize", &OCP_Point::initialize,
          bp::args("self", "x0", "oMtarget"))
      .def<void (OCP_Point::*)(const VectorXd)>(
          "solveFirst", &OCP_Point::solveFirst, bp::args("self", "x"))
      .def<void (OCP_Point::*)(const ConstVectorRef &)>(
          "solve", &OCP_Point::solve, bp::args("self", "x_measured"))
      .def("changeGoalCostActivation", &OCP_Point::changeGoalCostActivation,
           bp::args("index", "value"))
      .def("changeTarget", &OCP_Point::changeTarget,
           bp::args("index", "position"))
      .def("changeGoalTrackingWeights", &OCP_Point::changeGoaleTrackingWeights,
           bp::args("weight"))
      .def("changePostureReference", &OCP_Point::changePostureReference,
           bp::args("index", "reference"))
      .add_property("state", &OCP_Point::get_state)
      .add_property("solver", &OCP_Point::get_solver)
      .add_property("torque", &OCP_Point::get_torque,
                    "Torque command computed by the OCP")
      .add_property("gain", &OCP_Point::get_gain, "Gains computed by the OCP");
}

void exposeOCPPoint() {
  exposeOCPPointParams();
  exposeOCPPointClass();
}

}  // namespace python
}  // namespace mpc_p