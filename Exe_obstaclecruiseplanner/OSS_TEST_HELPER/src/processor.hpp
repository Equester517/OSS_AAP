#pragma once

#include <filesystem>
#include <random>

#include "jsonl_reader.hpp"

namespace oss_test_helper
{
class Processor
{
public:
  explicit Processor(std::filesystem::path project_root);
  void processScenario(int scenario_id);

private:
  std::filesystem::path findInputFile(const std::filesystem::path &scenario_dir) const;
  std::filesystem::path prepareOutputPath(const std::filesystem::path &input_file, int scenario_id) const;
  static std::string scenarioFolder(int scenario_id);

  std::filesystem::path project_root_;
  JsonlReader reader_;
  std::mt19937 rng_;
  std::uniform_int_distribution<int> interval_dist_{140, 188};
};
}  // namespace oss_test_helper
