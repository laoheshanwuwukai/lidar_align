
#include "lidar_align/common.h"
#include <fstream>
#include <iomanip>
#include <iostream>

void TransformTumPose(const fs::path &source_tum_pose_path,
                      const Eigen::Matrix4d &T_target_source,
                      const fs::path &output_path, bool init_from_I) {

  if (fs::exists(output_path)) {
    fs::remove(output_path);
  }

  std::ifstream fin;
  fin.open(source_tum_pose_path.string());
  std::ofstream fout;
  fout.open(output_path.string());
  std::string line;

  Eigen::Matrix4d T_source_target = T_target_source.inverse();

  std::cout << std::boolalpha << "Target tum pose start from Identity ? "
            << init_from_I << std::endl;

  bool init_target0_pose = init_from_I ? false : true;
  Eigen::Matrix4d T_target0_world = Eigen::Matrix4d::Identity();

  while (getline(fin, line)) {

    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::stringstream ss(line);

    double time, tx, ty, tz, qx, qy, qz, qw;
    ss >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    // from source to world
    Eigen::Quaterniond q_world_source(qw, qx, qy, qz);
    Eigen::Vector3d t_world_source(tx, ty, tz);
    Eigen::Matrix4d T_world_source = Eigen::Matrix4d ::Identity();
    T_world_source.block<3, 3>(0, 0) = q_world_source.matrix();
    T_world_source.block<3, 1>(0, 3) = t_world_source;

    // from target to world
    Eigen::Matrix4d T_world_target = T_world_source * T_source_target;
    if (!init_target0_pose) {
      std::cout << "Set from I" << std::endl;
      T_target0_world = T_world_target.inverse();
      init_target0_pose = true;
    }
    T_world_target = T_target0_world * T_world_target;
    Eigen::Quaterniond q(T_world_target.block<3, 3>(0, 0));
    Eigen::Vector3d t(T_world_target.block<3, 1>(0, 3));
    // clang-format off
    fout << std::setprecision(15) <<
            time  << " " <<
            t.x() << " " <<
            t.y() << " " <<
            t.z() << " " <<
            q.x() << " " <<
            q.y() << " " <<
            q.z() << " " <<
            q.w() << std::endl;
    // clang-format on
  }

  fin.close();
  fout.close();

  return;
}
