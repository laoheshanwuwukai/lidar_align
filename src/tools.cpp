
#include "lidar_align/common.h"
#include <Eigen/Dense>
#include <filesystem>
#include <gflags/gflags.h>

#include <fstream>
#include <iomanip>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;
DEFINE_double(dt, 0.0, "dt");

#define CHECK_FLOAT(val) checkFloat(val, #val)

template <typename T> void checkFloat(const T &value, const char *name) {
  if (std::is_same<T, float>::value) {
    std::cout << name << " is a float." << std::endl;
  } else {
    std::cout << name << " is NOT a float." << std::endl;
  }
}

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
  /*// For test*/
  /*double dt = FLAGS_dt;*/
  /*fs::path out =*/
  /*    lidar_folder.parent_path() / ("total" + std::to_string(dt) + ".pcd");*/
  /*// clang-format off*/
  /*Eigen::Matrix4d T_lidar_gj;*/
  /*T_lidar_gj <<*/
  /*  0-0.0265455, 0000.999412, 0-0.0216953, 00-0.830114,*/
  /*  00-0.999641, 0-0.0266193, -0.00311963, 00-0.165463,*/
  /*  -0.00369531, 000.0216047, 00000.99976, 00-0.878163,*/
  /*  000000000-0, 000000000-0, 000000000-0, 00000000001;*/
  /*// clang-format on*/
  /*Eigen::Matrix4d T_gj_lidar = T_lidar_gj.inverse();*/
  /**/
  /*CombainClouds(T_gj_lidar, lidar_folder, ins_file, out);*/
  /**/
  /*return 0;*/

  // float f = 1.66893005371094e-06;
  float f = 1.6893005371094e-06 * 57600;
  double d = f * 1e6;
  long long int result = static_cast<long long int>(d);
  float test = static_cast<long long int>(f * 1e6);

  std::cout << std::setprecision(15) << "f: " << f << "  d:" << d
            << "  lli: " << result << " test: " << test << std::endl;

  f = 99999;
  std::cout << "float " << f;
  return 0;
}
