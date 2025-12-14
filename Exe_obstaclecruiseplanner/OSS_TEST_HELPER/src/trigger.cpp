#include "trigger.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <iostream>

namespace oss_test_helper
{
Trigger::Trigger(Processor &processor) : processor_(processor) {}

void Trigger::run()
{
  std::cout << "waiting for scenario command (scenario_1..scenario_3, exit)\n";
  std::string line;
  while (std::cout << "> " && std::getline(std::cin, line))
  {
    const auto trimmed = trim(line);
    if (trimmed.empty())
    {
      continue;
    }
    if (trimmed == "exit" || trimmed == "quit")
    {
      break;
    }
    int scenario_id = 0;
    if (!parseScenario(trimmed, scenario_id))
    {
      std::cout << "unknown command: " << trimmed << '\n';
      continue;
    }

    std::cout << trimmed << " triggered\n";
    try
    {
      processor_.processScenario(scenario_id);
    }
    catch (const std::exception &e)
    {
      std::cerr << "[Processor] error: " << e.what() << '\n';
    }
  }
  std::cout << "trigger loop exiting\n";
}

std::string Trigger::trim(const std::string &value)
{
  const auto front = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch);
  });
  const auto back = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch);
  }).base();
  if (front >= back)
  {
    return {};
  }
  return std::string(front, back);
}

bool Trigger::parseScenario(const std::string &text, int &out_id)
{
  constexpr const char prefix[] = "scenario_";
  if (text.rfind(prefix, 0) != 0)
  {
    return false;
  }
  const auto suffix = text.substr(sizeof(prefix) - 1);
  try
  {
    const int value = std::stoi(suffix);
    if (value < 1 || value > 3)
    {
      return false;
    }
    out_id = value;
    return true;
  }
  catch (const std::exception &)
  {
    return false;
  }
}
}  // namespace oss_test_helper
