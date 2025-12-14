#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace oss_test_helper
{
namespace detail
{
inline void serializeValue(const nlohmann::ordered_json &value, std::string &output)
{
  if (value.is_object())
  {
    output.push_back('{');
    bool first_element = true;
    for (const auto &item : value.items())
    {
      if (!first_element)
      {
        output += ", ";
      }
      first_element = false;
      nlohmann::ordered_json key_json(item.key());
      output += key_json.dump();
      output += ": ";
      serializeValue(item.value(), output);
    }
    output.push_back('}');
  }
  else if (value.is_array())
  {
    output.push_back('[');
    bool first_element = true;
    for (const auto &element : value)
    {
      if (!first_element)
      {
        output += ", ";
      }
      first_element = false;
      serializeValue(element, output);
    }
    output.push_back(']');
  }
  else if (value.is_string())
  {
    output += value.dump();
  }
  else if (value.is_boolean())
  {
    output += value.get<bool>() ? "true" : "false";
  }
  else if (value.is_number())
  {
    output += value.dump();
  }
  else if (value.is_null())
  {
    output += "null";
  }
}
}  // namespace detail

inline std::string serializeOrderedJsonSingleLine(const nlohmann::ordered_json &value)
{
  std::string result;
  detail::serializeValue(value, result);
  return result;
}
}  // namespace oss_test_helper
