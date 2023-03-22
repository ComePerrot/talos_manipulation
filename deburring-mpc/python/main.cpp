#include <eigenpy/eigenpy.hpp>

#include "deburring_mpc/python-fwd.hpp"

BOOST_PYTHON_MODULE(deburring_mpc_pywrap) {
  boost::python::import("pinocchio");
  boost::python::import("crocoddyl");
  // Enabling eigenpy support, i.e. numpy/eigen compatibility.
  eigenpy::enableEigenPy();
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::VectorXi);
  mpc_p::python::exposeDesigner();
  mpc_p::python::exposeOCPPoint();
  mpc_p::python::exposeMPCPoint();
}