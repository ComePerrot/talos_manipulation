#include <yaml-cpp/yaml.h>

#include "deburring_mpc/mpc.hpp"

namespace mpc_p {

void MPC_Point::logData(const Eigen::Ref<const Eigen::VectorXd> x_input,
                        const Eigen::Ref<const Eigen::VectorXd> us0,
                        const Eigen::Ref<const Eigen::MatrixXd> K0) {
  MPC_debugData data;
  data.x_input = x_input;

  MPC_command command;
  command.us0 = us0;
  command.K0 = K0;

  data.output = command;

  debugDataMPC_.push_back(data);
}

void MPC_Point::dumpToFile(std::string name) {
  std::ofstream ofs(name.c_str());
  boost::archive::text_oarchive oa(ofs);
  oa << debugDataMPC_;
}

std::vector<MPC_debugData> MPC_Point::fetchFromFile(std::string name) {
  std::vector<MPC_debugData> datas;

  std::ifstream ifs(name.c_str());
  boost::archive::text_iarchive ia(ifs);
  ia >> datas;

  return (datas);
}

}  // namespace mpc_p