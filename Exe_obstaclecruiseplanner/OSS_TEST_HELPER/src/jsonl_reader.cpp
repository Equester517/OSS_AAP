#include "jsonl_reader.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace oss_test_helper
{
std::vector<JsonTrajectoryRecord> JsonlReader::read(const std::filesystem::path &input_path) const
{
  std::ifstream stream(input_path);
  if (!stream)
  {
    throw std::runtime_error("failed to open input jsonl: " + input_path.string());
  }

  std::vector<JsonTrajectoryRecord> records;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(stream, line))
  {
    ++line_number;
    if (line.empty())
    {
      continue;
    }
    try
    {
      nlohmann::ordered_json parsed = nlohmann::ordered_json::parse(line);
      records.push_back({parsed, parseTrajectory(parsed)});
    }
    catch (const std::exception &e)
    {
      throw std::runtime_error("failed to parse line " + std::to_string(line_number) + ": " + e.what());
    }
  }
  return records;
}

Trajectory JsonlReader::parseTrajectory(const nlohmann::ordered_json &root)
{
  Trajectory trajectory;
  const auto message_json = root.value("message", nlohmann::ordered_json::object());
  const auto header_json = message_json.value("header", nlohmann::ordered_json::object());
  const auto stamp_json = header_json.value("stamp", nlohmann::ordered_json::object());

  trajectory.header.seq = header_json.value("seq", 0u);
  trajectory.header.frame_id = header_json.value("frame_id", std::string{});
  trajectory.header.stamp.sec = stamp_json.value("sec", 0ll);
  trajectory.header.stamp.nanosec = stamp_json.value("nanosec", 0ll);

  const auto points_json = message_json.value("points", nlohmann::ordered_json::array());
  trajectory.points.reserve(points_json.size());
  for (const auto &point_json : points_json)
  {
    trajectory.points.push_back(parsePoint(point_json));
  }
  return trajectory;
}

TrajectoryPoint JsonlReader::parsePoint(const nlohmann::ordered_json &point_json)
{
  TrajectoryPoint point;
  point.time_from_start = parseDuration(point_json.value("time_from_start", nlohmann::ordered_json::object()));
  point.pose = parsePose(point_json.value("pose", nlohmann::ordered_json::object()));
  point.longitudinal_velocity_mps = point_json.value("longitudinal_velocity_mps", 0.0);
  point.lateral_velocity_mps = point_json.value("lateral_velocity_mps", 0.0);
  point.acceleration_mps2 = point_json.value("acceleration_mps2", 0.0);
  point.heading_rate_rps = point_json.value("heading_rate_rps", 0.0);
  point.front_wheel_angle_rad = point_json.value("front_wheel_angle_rad", 0.0);
  point.rear_wheel_angle_rad = point_json.value("rear_wheel_angle_rad", 0.0);
  return point;
}

Duration JsonlReader::parseDuration(const nlohmann::ordered_json &duration_json)
{
  Duration duration;
  duration.sec = duration_json.value("sec", 0);
  duration.nanosec = duration_json.value("nanosec", 0u);
  return duration;
}

Pose JsonlReader::parsePose(const nlohmann::ordered_json &pose_json)
{
  Pose pose;
  pose.position = parsePosition(pose_json.value("position", nlohmann::ordered_json::object()));
  pose.orientation = parseOrientation(pose_json.value("orientation", nlohmann::ordered_json::object()));
  return pose;
}

Position JsonlReader::parsePosition(const nlohmann::ordered_json &position_json)
{
  Position position;
  position.x = position_json.value("x", 0.0);
  position.y = position_json.value("y", 0.0);
  position.z = position_json.value("z", 0.0);
  return position;
}

Orientation JsonlReader::parseOrientation(const nlohmann::ordered_json &orientation_json)
{
  Orientation orientation;
  orientation.x = orientation_json.value("x", 0.0);
  orientation.y = orientation_json.value("y", 0.0);
  orientation.z = orientation_json.value("z", 0.0);
  orientation.w = orientation_json.value("w", 1.0);
  return orientation;
}
}  // namespace oss_test_helper
