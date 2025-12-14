#include "jsonl_writer.hpp"
#include "pretty_json_single_line_serializer.hpp"

#include <stdexcept>
#include <string>

namespace
{
using ordered_json = nlohmann::ordered_json;

ordered_json normalizeEntry(const ordered_json &entry)
{
  ordered_json normalized;
  normalized["topic"] = entry.value("topic", std::string{});
  normalized["timestamp"] = entry.value("timestamp", 0ll);
  auto &normalized_message = normalized["message"];
  normalized_message = ordered_json::object();

  const ordered_json message = entry.value("message", ordered_json::object());
  const ordered_json header_json = message.value("header", ordered_json::object());
  const ordered_json stamp_json = header_json.value("stamp", ordered_json::object());

  ordered_json stamp;
  stamp["sec"] = stamp_json.value("sec", 0ll);
  stamp["nanosec"] = stamp_json.value("nanosec", 0ll);

  ordered_json header;
  header["stamp"] = stamp;
  header["frame_id"] = header_json.value("frame_id", std::string{});
  if (header_json.contains("seq"))
  {
    header["seq"] = header_json["seq"];
  }

  normalized_message["header"] = header;
  for (const auto &item : message.items())
  {
    if (item.key() == "header")
    {
      continue;
    }
    normalized_message[item.key()] = item.value();
  }

  return normalized;
}

void verifyLineFormat(const std::string &line)
{
  if (line.rfind("{\"topic\": ", 0) != 0)
  {
    throw std::runtime_error("jsonl line does not start with expected topic order");
  }

  const auto topic_pos = line.find("\"topic\"");
  const auto timestamp_pos = line.find("\"timestamp\"");
  const auto message_pos = line.find("\"message\"");
  if (topic_pos == std::string::npos || timestamp_pos == std::string::npos || message_pos == std::string::npos ||
      !(topic_pos < timestamp_pos && timestamp_pos < message_pos))
  {
    throw std::runtime_error("jsonl line violates the expected top-level key order");
  }

  if (line.find("\": ") == std::string::npos || line.find(", ") == std::string::npos)
  {
    throw std::runtime_error("jsonl line is missing the expected whitespace separators");
  }

  const auto header_pos = line.find("\"header\"");
  const auto stamp_pos = line.find("\"stamp\"");
  const auto frame_id_pos = line.find("\"frame_id\"");
  if (header_pos == std::string::npos || stamp_pos == std::string::npos || frame_id_pos == std::string::npos ||
      !(stamp_pos < frame_id_pos))
  {
    throw std::runtime_error("jsonl line violates the expected header key order");
  }

  const auto sec_pos = line.find("\"sec\"");
  const auto nanosec_pos = line.find("\"nanosec\"");
  if (sec_pos == std::string::npos || nanosec_pos == std::string::npos || sec_pos > nanosec_pos)
  {
    throw std::runtime_error("jsonl line does not keep stamp.sec before stamp.nanosec");
  }
}
}  // namespace

namespace oss_test_helper
{
JsonlWriter::JsonlWriter(const std::filesystem::path &output_path)
{
  stream_.open(output_path, std::ios::out | std::ios::trunc);
  if (!stream_)
  {
    throw std::runtime_error("failed to open output file: " + output_path.string());
  }
}

void JsonlWriter::append(const ordered_json &entry)
{
  const auto normalized = normalizeEntry(entry);
  const auto line = serializeOrderedJsonSingleLine(normalized);
  stream_ << line << '\n';
  if (!stream_)
  {
    throw std::runtime_error("failed to write jsonl entry");
  }

  if (verified_lines_ < 3)
  {
    verifyLineFormat(line);
    ++verified_lines_;
  }
}
}  // namespace oss_test_helper
