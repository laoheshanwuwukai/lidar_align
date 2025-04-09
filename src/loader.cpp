
#include <cstdint>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <sstream>

#include "lidar_align/common.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"
#include "lidar_align/transform.h"

namespace lidar_align {

Loader::Loader(const Config &config) : config_(config) {}

Loader::Config Loader::getConfig(const YAML::Node &node) {
  Loader::Config config;
  const std::string loader = "loader";
  config.use_n_scans = _ReadYaml<int>(node, {loader, "use_n_scans"});
  return config;
}

bool Loader::loadPointCloudFromFolder(const fs::path &folder,
                                      const Scan::Config &scan_config,
                                      Lidar *lidar) {
  if (!fs::exists(folder) || !fs::is_directory(folder)) {
    std::cout << "Error: input pointcloud from folder failed ->"
              << folder.string();
    return false;
  }

  std::vector<fs::path> all_cloud_files = getAllfiles(folder, ".pcd");
  std::sort(all_cloud_files.begin(), all_cloud_files.end());

  std::cout << "Total cloud file : " << all_cloud_files.size();
  for (const auto &f : all_cloud_files) {
    LoaderPointcloud pointcloud;
    // first load origin cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr origin_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    if (-1 == pcl::io::loadPCDFile(f, *origin_cloud)) {
      continue;
    }

    std::string name = f.stem().string();
    std::uint64_t time = std::stoull(name);
    pointcloud.header.stamp = time;
    for (const auto &p : origin_cloud->points) {
      PointAllFields point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      pointcloud.emplace_back(std::move(point));
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
    }

    std::cout << "point cloud time : " << pointcloud.header.stamp;
    lidar->addPointcloud(pointcloud, scan_config);
    if (lidar->getNumberOfScans() >= config_.use_n_scans) {
      break;
    }
  }

  /*if (lidar->getTotalPoints() == 0) {*/
  /*  std::cout << "No points were loaded, verify that the folder";*/
  /*  return false;*/
  /*}*/

  return true;
}
bool Loader::loadTfromFile(const fs::path &file, Odom *odom) {
  if (!fs::exists(file)) {
    std::cout << "Error: input sensor from file failed -> " << file.string();
    return false;
  }
  std::ifstream f(file.string());
  std::string line;
  while (getline(f, line)) {
    std::stringstream ss(line);
    Timestamp stamp;
    ss >> stamp;
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
    // clang-format off
    ss >>
      Ti(0,0)>> Ti(0,1)>> Ti(0,2)>> Ti(0,3)>>
      Ti(1,0)>> Ti(1,1)>> Ti(1,2)>> Ti(1,3)>>
      Ti(2,0)>> Ti(2,1)>> Ti(2,2)>> Ti(2,3);
    // clang-format on
    Eigen::Quaternionf q(Ti.block<3, 3>(0, 0));
    Eigen::Vector3f t(Ti.block<3, 1>(0, 3));
    lidar_align::Transform T(t, q);
    odom->addTransformData(stamp, T);
  }

  if (odom->empty()) {
    std::cout << "No odom message found";
    return false;
  }

  return true;
}

} // namespace lidar_align
