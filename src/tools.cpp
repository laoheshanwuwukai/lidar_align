
#include <Eigen/Dense>
#include <filesystem>
#include <gflags/gflags.h>

namespace fs = std::filesystem;

void Combainundistortcloud(const fs::path &lidar_folder,
                           const fs::path &ins_file, const fs::path &out) {

  return;
}

int main(int argc, char *argv[]) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  return 0;
}
