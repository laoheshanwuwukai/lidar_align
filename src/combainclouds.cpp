
#include "lidar_align/common.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <gflags/gflags.h>

DEFINE_string(config, "/home/udeer/lidar_align/config/config.yaml",
              "config yaml file");
DEFINE_string(lidar, "", "input lidar folder");
DEFINE_string(ins, "", "input ins data file");

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
  fs::path config(FLAGS_config);
  if (!fs::exists(config)) {
    std::cout << "input config file error -> " << config.string();
    return 0;
  }
  const YAML::Node node = YAML::LoadFile(config.string());

  const auto debug_config = GetDebugConfig(node);
  std::cout << debug_config.T_target_source << std::endl;

  fs::path lidar_folder(FLAGS_lidar);
  fs::path ins_file(FLAGS_ins);

  // origin things
  const lidar_align::Scan::Config scan_config =
      lidar_align::Scan::getYAMLConfig(node);
  lidar_align::Lidar lidar;
  lidar_align::Odom odom;
  lidar_align::Loader loader(lidar_align::Loader::getConfig(node));
  // loader.loadTfromFile(ins_file, &odom);
  loader.loadTumTfromFile(ins_file, &odom);
  loader.loadPointCloudFromFolder(lidar_folder, scan_config, &lidar);
  lidar.setOdomOdomTransforms(odom);
  // lataest things
  const auto T_target_source = debug_config.T_target_source.cast<float>();
  Eigen::Quaternionf q(T_target_source.block<3, 3>(0, 0));
  Eigen::Vector3f t(T_target_source.block<3, 1>(0, 3));

  lidar_align::Transform T_o_l(t, q);
  lidar.setOdomLidarTransform(T_o_l);
  std::string name = "total.ply";

  fs::path save_path = lidar_folder.parent_path() / name;
  lidar.saveCombinedPointcloud(save_path.string());
  std::cout << std::endl;
  std::cout << "Save final result to " << save_path.string();
  std::cout << std::endl;

  return 0;
}
