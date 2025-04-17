#pragma once
#include <Eigen/Dense>
#include <filesystem>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

/*
 * Recursively read values from YAML(according to keys)
 */
template <typename T>
T _ReadYaml(const YAML::Node &node, const std::vector<std::string> &keys,
            std::size_t index = 0) {
  if (keys.empty()) {
    throw std::runtime_error("Key vector is empty");
  }
  YAML::Node current = node[keys[index]];
  if (!current) {
    throw std::runtime_error("Key not found: " + keys[index]);
  }

  if (index == keys.size() - 1) {
    return current.as<T>();
  } else {
    return _ReadYaml<T>(current, keys, index + 1);
  }
}

static std::vector<fs::path> getAllfiles(fs::path folder_path,
                                         std::string extension) {
  std::vector<fs::path> files;
  for (const auto &entry : fs::directory_iterator(folder_path)) {
    if (entry.is_regular_file() && entry.path().extension() == extension) {
      files.push_back(entry.path());
    }
  }
  return files;
}

void TransformTumPose(const fs::path &source_tum_pose_path,
                      const Eigen::Matrix4d &T_target_source,
                      const fs::path &output_path, bool init_from_I = true);
