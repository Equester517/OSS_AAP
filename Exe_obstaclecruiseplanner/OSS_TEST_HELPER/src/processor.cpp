#include "processor.hpp"

#include "jsonl_writer.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <fstream>
#include <thread>

#include <nlohmann/json.hpp>

namespace
{
constexpr std::int64_t kNanosecondsInSecond = 1'000'000'000;

void updateJsonTimestamp(nlohmann::ordered_json &record, std::int64_t timestamp_ns)
{
  record["timestamp"] = timestamp_ns;
  auto &stamp = record["message"]["header"]["stamp"];
  stamp["sec"] = timestamp_ns / kNanosecondsInSecond;
  stamp["nanosec"] = timestamp_ns % kNanosecondsInSecond;
}

std::string readFirstNonEmptyLine(const std::filesystem::path &path)
{
  std::ifstream stream(path);
  if (!stream)
  {
    return {};
  }
  std::string line;
  while (std::getline(stream, line))
  {
    if (!line.empty())
    {
      return line;
    }
  }
  return {};
}

std::string snippet(const std::string &line, std::size_t limit = 200)
{
  if (line.size() <= limit)
  {
    return line;
  }
  return line.substr(0, limit);
}
}  // namespace

namespace oss_test_helper
{
Processor::Processor(std::filesystem::path project_root)
  : project_root_(std::move(project_root)), rng_(std::random_device{}())
{
}

void Processor::processScenario(int scenario_id)
{
  const auto scenario_name = scenarioFolder(scenario_id);
  const auto scenario_dir = project_root_ / "reference_output" / scenario_name;
  if (!std::filesystem::exists(scenario_dir))
  {
    throw std::runtime_error("reference directory missing: " + scenario_dir.string());
  }

  const auto input_file = findInputFile(scenario_dir);
  std::cout << "[Processor] scenario_" << scenario_id << " reading " << input_file << "\n";

  const auto output_dir = project_root_ / "output";
  std::filesystem::create_directories(output_dir);
  const auto output_file = prepareOutputPath(input_file.filename(), scenario_id);

  auto records = reader_.read(input_file);

  std::cout << "[Processor] processing " << records.size() << " messages\n";
  std::int64_t current_ts_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();

  {
    JsonlWriter writer(output_file);
    for (std::size_t index = 0; index < records.size(); ++index)
    {
      auto &record = records[index];
      updateJsonTimestamp(record.raw, current_ts_ns);

      const auto sec = current_ts_ns / kNanosecondsInSecond;
      const auto nanosec = current_ts_ns % kNanosecondsInSecond;
      record.trajectory.header.stamp.sec = sec;
      record.trajectory.header.stamp.nanosec = nanosec;

      std::cout << "  [message " << (index + 1) << "] ts=" << current_ts_ns
                << " points=" << record.trajectory.points.size() << "\n";

      writer.append(record.raw);

      if (index + 1 < records.size())
      {
        const int delay_ms = interval_dist_(rng_);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        current_ts_ns += static_cast<std::int64_t>(delay_ms) * 1'000'000;
      }
    }
  }

  const auto reference_sample = readFirstNonEmptyLine(input_file);
  if (!reference_sample.empty())
  {
    std::cout << "[Processor] reference first line snippet: " << snippet(reference_sample) << "\n";
  }
  const auto output_sample = readFirstNonEmptyLine(output_file);
  if (!output_sample.empty())
  {
    std::cout << "[Processor] output first line snippet: " << snippet(output_sample) << "\n";
  }
  std::cout << "[Processor] output saved to " << output_file << "\n";
}

std::filesystem::path Processor::findInputFile(const std::filesystem::path &scenario_dir) const
{
  for (const auto &entry : std::filesystem::directory_iterator(scenario_dir))
  {
    if (!entry.is_regular_file())
    {
      continue;
    }
    const auto name = entry.path().filename().string();
    if (name.rfind("planning__scenario_planning", 0) == 0)
    {
      return entry.path();
    }
  }
  throw std::runtime_error("no planning file found in " + scenario_dir.string());
}

std::filesystem::path Processor::prepareOutputPath(const std::filesystem::path &input_file, int scenario_id) const
{
  const auto stem = input_file.stem().string();
  const auto extension = input_file.extension().string();
  const std::string suffix = stem + "_scenario_" + std::to_string(scenario_id) + extension;
  return (project_root_ / "output") / suffix;
}

std::string Processor::scenarioFolder(int scenario_id)
{
  return "scenario_" + std::to_string(scenario_id);
}
}  // namespace oss_test_helper
