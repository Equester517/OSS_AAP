#include "processor.hpp"
#include "trigger.hpp"

#include <filesystem>
#include <iostream>

namespace
{
std::filesystem::path detectProjectRoot()
{
  const auto candidate = std::filesystem::current_path();
  if (std::filesystem::exists(candidate / "reference_output"))
  {
    return candidate;
  }

  const auto source_file = std::filesystem::absolute(std::filesystem::path(__FILE__));
  const auto fallback = source_file.parent_path().parent_path();
  if (std::filesystem::exists(fallback / "reference_output"))
  {
    return fallback;
  }

  return candidate;
}
}  // namespace

int main()
{
  const auto project_root = detectProjectRoot();
  std::cout << "[Main] using project root: " << project_root << '\n';
  oss_test_helper::Processor processor(project_root);
  oss_test_helper::Trigger trigger(processor);
  trigger.run();
  return 0;
}
