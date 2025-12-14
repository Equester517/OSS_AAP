#pragma once

#include <filesystem>
#include <vector>

#include <nlohmann/json.hpp>

#include "types.hpp"

namespace oss_test_helper
{
struct JsonTrajectoryRecord
{
  nlohmann::ordered_json raw;
  Trajectory trajectory;
};

class JsonlReader
{
public:
  std::vector<JsonTrajectoryRecord> read(const std::filesystem::path &input_path) const;

private:
  static Trajectory parseTrajectory(const nlohmann::ordered_json &root);
  static TrajectoryPoint parsePoint(const nlohmann::ordered_json &point_json);
  static Duration parseDuration(const nlohmann::ordered_json &duration_json);
  static Pose parsePose(const nlohmann::ordered_json &pose_json);
  static Position parsePosition(const nlohmann::ordered_json &position_json);
  static Orientation parseOrientation(const nlohmann::ordered_json &orientation_json);
};
}  // namespace oss_test_helper
