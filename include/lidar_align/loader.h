#ifndef LIDAR_ALIGN_LOADER_H_
#define LIDAR_ALIGN_LOADER_H_

#include <filesystem>
#include <pcl/point_types.h>

#include "lidar_align/sensors.h"

#define PCL_NO_PRECOMPILE

namespace lidar_align {
namespace fs = std::filesystem;

class Loader {
public:
  struct Config {
    int use_n_scans = std::numeric_limits<int>::max();
  };

  Loader(const Config &config);

  /*static Config getConfig(ros::NodeHandle *nh);*/
  /*void parsePointcloudMsg(const sensor_msgs::PointCloud2 msg,*/
  /*                        LoaderPointcloud *pointcloud);*/
  /*bool loadTformFromMaplabCSV(const std::string &csv_path, Odom *odom);*/

  // TODO origin load pointcloud and odom from rosbag
  /*bool loadPointcloudFromROSBag(const std::string &bag_path,*/
  /*                              const Scan::Config &scan_config, Lidar
   * *lidar);*/
  /*bool loadTformFromROSBag(const std::string &bag_path, Odom *odom);*/

  bool loadPointCloudFromFolder(const fs::path &folder,
                                const Scan::Config &scan_config, Lidar *lidar);

  // Load pose from T matrix file
  bool loadTfromFile(const fs::path &file, Odom *odom);

  // Load pose from tum format file
  bool loadTumTfromFile(const fs::path &file, Odom *odom);

  static Config getConfig(const YAML::Node &node);

private:
  /*static bool getNextCSVTransform(std::istream &str, Timestamp *stamp,*/
  /*                                Transform *T);*/
  Timestamp first_odom_us;

  Config config_;
};
} // namespace lidar_align

POINT_CLOUD_REGISTER_POINT_STRUCT(
    lidar_align::PointAllFields,
    (float, x, x)(float, y, y)(float, z, z)(int32_t, time_offset_us,
                                            time_offset_us)(
        uint16_t, reflectivity, reflectivity)(uint16_t, intensity,
                                              intensity)(uint8_t, ring, ring))

#endif // LIDAR_ALIGN_ALIGNER_H_
