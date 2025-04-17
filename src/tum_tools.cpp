
#include "lidar_align/common.h"
#include <Eigen/Dense>
#include <gflags/gflags.h>
#include <yaml-cpp/node/parse.h>
namespace fs = std::filesystem;

DEFINE_string(config, "/home/udeer/lidar_align/config/config.yaml",
              "config yaml file");
DEFINE_string(pose, "", "input ins data file");
DEFINE_string(out, "", "output_path");
DEFINE_bool(i, true, " Init from identity");

struct TumPoseConfig {
  Eigen::Matrix4d T_source_target;
  Eigen::Matrix4d T_target_source;
};

TumPoseConfig GetDebugConfig(const YAML::Node &node) {
  const std::string prefix = "debug";
  TumPoseConfig conf;
  auto vec = _ReadYaml<std::vector<double>>(node, {prefix, "T_target_source"});
  auto evo = _ReadYaml<std::vector<double>>(node, {prefix, "evo_result"});
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_target_source(
      vec.data());
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
      T_target_refinedtarget(evo.data());
  conf.T_target_source = T_target_refinedtarget.inverse() * T_target_source;
  conf.T_source_target = conf.T_target_source.inverse();
  return conf;
}

int main(int argc, char *argv[]) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  fs::path config_file(FLAGS_config);
  fs::path pose_file(FLAGS_pose);
  fs::path out_file(FLAGS_out);
  YAML::Node node = YAML::LoadFile(config_file.string());
  const auto &conf = GetDebugConfig(node);

  TransformTumPose(pose_file, conf.T_target_source, out_file, FLAGS_i);

  return 0;
}
