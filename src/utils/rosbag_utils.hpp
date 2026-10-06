#pragma once

#include <string>

namespace gopro_ros {

struct BagConfig {
  std::string uri;
  std::string storage_id;
};

/**
 * @brief Normalize the bag URI and ROS storage backend from an explicit storage selection.
 *
 * Supported storage_id values:
 *   - ".mcap" -> storage_id = "mcap"
 *   - ".db3"  -> storage_id = "sqlite3"
 *
 * The provided bag_path is treated as the bag directory path. The storage backend
 * will create the actual bag files inside it (for example: <dir>/bag_0.mcap and
 * <dir>/metadata.yaml). Any unsupported value falls back to ".db3" behavior.
 *
 * @param bag_path input bag directory path (may or may not have an extension)
 * @param storage_id storage selector (".mcap" or ".db3")
 * @return BagConfig containing uri and storage_id
 */
BagConfig inferBagConfig(const std::string& bag_path, const std::string& storage_id = ".db3");

}  // namespace gopro_ros
