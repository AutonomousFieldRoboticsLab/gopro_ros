#include "utils/rosbag_utils.hpp"

#include <filesystem>

namespace gopro_ros {

namespace fs = std::filesystem;

BagConfig inferBagConfig(const std::string& bag_path, const std::string& storage_id) {
  fs::path p(bag_path);
  BagConfig cfg;

  cfg.storage_id = "sqlite3";
  if (storage_id == ".mcap") {
    cfg.storage_id = "mcap";
  } else if (storage_id == ".db3") {
    cfg.storage_id = "sqlite3";
  }

  // Treat the input as a bag directory path. The storage backend will create the
  // actual bag files inside it (for example: <dir>/bag_0.mcap and <dir>/metadata.yaml).
  const auto ext = p.extension().string();
  if (ext == ".db3" || ext == ".mcap") {
    p.replace_extension("");
  }

  cfg.uri = p.string();

  return cfg;
}

}  // namespace gopro_ros
