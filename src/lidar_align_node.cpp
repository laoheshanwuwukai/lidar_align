#include <gflags/gflags.h>

#include "lidar_align/aligner.h"
#include "lidar_align/common.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"

DEFINE_string(config, "", "config yaml file");
DEFINE_string(lidar, "", "input lidar folder");
DEFINE_string(ins, "", "input ins data file");
namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  fs::path config(FLAGS_config);
  if (!fs::exists(config)) {
    std::cout << "input config file error -> " << config.string();
    return 0;
  }
  const YAML::Node node = YAML::LoadFile(config.string());

  fs::path lidar_folder(FLAGS_lidar);
  fs::path ins_file(FLAGS_ins);

  const lidar_align::Scan::Config scan_config =
      lidar_align::Scan::getYAMLConfig(node);
  lidar_align::Lidar lidar;
  lidar_align::Odom odom;
  lidar_align::Loader loader(lidar_align::Loader::getConfig(node));

  if (!loader.loadTfromFile(ins_file, &odom)) {
    std::cout << "Error: loader load T failed";
    return -1;
  }

  /*if (!loader.loadTumTfromFile(ins_file, &odom)) {*/
  /*  std::cout << "Error: loader load T failed";*/
  /*  return -1;*/
  /*}*/

  if (!loader.loadPointCloudFromFolder(lidar_folder, scan_config, &lidar)) {
    std::cout << "Error: loader load lidar failed";
    return -1;
  }
  std::cout << "Load lidar finished" << std::endl;

  std::cout << "Load odom finished" << std::endl;
  lidar.setOdomOdomTransforms(odom);

  lidar_align::Aligner aligner(lidar_align::getConfig(node));

  aligner.lidarOdomTransform(&lidar, &odom);

  return 0;
}
