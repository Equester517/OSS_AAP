#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

namespace oss_test_helper
{
class JsonlWriter
{
public:
  explicit JsonlWriter(const std::filesystem::path &output_path);
  void append(const nlohmann::ordered_json &entry);

private:
  std::ofstream stream_;
  std::size_t verified_lines_{0};
};
}  // namespace oss_test_helper
