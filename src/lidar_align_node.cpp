#include <gflags/gflags.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "lidar_align/aligner.h"
#include "lidar_align/common.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"

DEFINE_string(config, "", "config yaml file");
DEFINE_string(lidar, "", "input lidar folder");
DEFINE_string(ins, "", "input ins data file");
DEFINE_double(dt, 0.0, "dt");
namespace fs = std::filesystem;

void CombainClouds(const Eigen::Matrix4d &T_ins_lidar,
                   const fs::path &lidar_folder, const fs::path &ins_file,
                   const fs::path &out) {

  // Load all lidar time;
  auto all_lidar_files = getAllfiles(lidar_folder, ".pcd");
  std::sort(all_lidar_files.begin(), all_lidar_files.end());

  std::unordered_map<fs::path, Eigen::Matrix4d> cloud_T;

  std::ifstream file(ins_file);
  std::string line;
  std::size_t index = 0;

  while (getline(file, line) && index < all_lidar_files.size()) {
    std::stringstream ss(line);
    double ts_lidar = std::stod(all_lidar_files[index].stem().string());
    uint64_t tus_ins;
    ss >> tus_ins;
    double ts_ins = static_cast<double>(tus_ins) / 1e6;
    if (ts_ins < ts_lidar) {
      continue;
    } else {
      std::cout << "lidar_time - ins_time (s) : " << ts_lidar - ts_ins
                << std::endl;
      Eigen::Matrix4d T_world_ins = Eigen::Matrix4d::Identity();
      // clang-format off
      ss >>
        T_world_ins(0, 0) >> T_world_ins(0, 1) >> T_world_ins(0, 2) >> T_world_ins(0, 3) >>
        T_world_ins(1, 0) >> T_world_ins(1, 1) >> T_world_ins(1, 2) >> T_world_ins(1, 3) >>
        T_world_ins(2, 0) >> T_world_ins(2, 1) >> T_world_ins(2, 2) >> T_world_ins(2, 3);
      // clang-format on

      Eigen::Matrix4d T_world_lidar = T_world_ins * T_ins_lidar;
      cloud_T[all_lidar_files[index]] = T_world_lidar;

      index += 10;
    }

  } // end while

  pcl::PointCloud<pcl::PointXYZI>::Ptr result(
      new pcl::PointCloud<pcl::PointXYZI>());
  for (const auto &[f, T] : cloud_T) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr origin_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());

    if (-1 == pcl::io::loadPCDFile(f.string(), *origin_cloud)) {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>());

    pcl::transformPointCloud(*origin_cloud, *cloud, T);

    *result += *cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledCloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
  double leaf_size = 0.1;
  voxelGrid.setInputCloud(result);
  voxelGrid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxelGrid.filter(*downsampledCloud);
  pcl::io::savePCDFile(out.string(), *downsampledCloud);

  return;
}

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

  // For test
  double dt = FLAGS_dt;
  fs::path out =
      lidar_folder.parent_path() / ("total" + std::to_string(dt) + ".pcd");
  // clang-format off
  Eigen::Matrix4d T_lidar_gj;
  T_lidar_gj <<
    0-0.0265455, 0000.999412, 0-0.0216953, 00-0.830114,
    00-0.999641, 0-0.0266193, -0.00311963, 00-0.165463,
    -0.00369531, 000.0216047, 00000.99976, 00-0.878163,
    000000000-0, 000000000-0, 000000000-0, 00000000001;
  // clang-format on
  Eigen::Matrix4d T_gj_lidar = T_lidar_gj.inverse();

  CombainClouds(T_gj_lidar, lidar_folder, ins_file, out);

  return 0;

  const lidar_align::Scan::Config scan_config =
      lidar_align::Scan::getYAMLConfig(node);
  lidar_align::Lidar lidar;
  lidar_align::Odom odom;
  lidar_align::Loader loader(lidar_align::Loader::getConfig(node));

  if (!loader.loadPointCloudFromFolder(lidar_folder, scan_config, &lidar)) {
    std::cout << "Error: loader load lidar failed";
    return -1;
  }
  std::cout << "Load lidar finished" << std::endl;

  if (!loader.loadTfromFile(ins_file, &odom)) {
    std::cout << "Error: loader load T failed";
    return -1;
  }
  std::cout << "Load odom finished" << std::endl;
  lidar.setOdomOdomTransforms(odom);

  lidar_align::Aligner aligner(lidar_align::getConfig(node));

  aligner.lidarOdomTransform(&lidar, &odom);

  return 0;
}
