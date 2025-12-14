#pragma once

#include <string>

#include "processor.hpp"

namespace oss_test_helper
{
class Trigger
{
public:
  explicit Trigger(Processor &processor);
  void run();

private:
  static std::string trim(const std::string &value);
  static bool parseScenario(const std::string &text, int &out_id);

  Processor &processor_;
};
}  // namespace oss_test_helper
