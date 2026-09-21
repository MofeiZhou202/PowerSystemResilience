#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "mipsolvers/l2o/trace_event.hpp"

namespace mipsolvers::l2o {

struct TraceWriterOptions {
  bool append{false};
  bool flush_each_event{true};
  bool write_header{true};
  std::string run_id;
  std::string trace_schema_version{"mipsolvers.l2o.trace.v1"};
  nlohmann::json metadata = nlohmann::json::object();
};

class TraceWriter {
 public:
  TraceWriter(const std::filesystem::path& path,
              TraceWriterOptions options = TraceWriterOptions{});
  ~TraceWriter();

  TraceWriter(const TraceWriter&) = delete;
  TraceWriter& operator=(const TraceWriter&) = delete;
  TraceWriter(TraceWriter&&) noexcept = default;
  TraceWriter& operator=(TraceWriter&&) noexcept = default;

  bool is_open() const;
  const std::filesystem::path& path() const;
  std::uint64_t events_written() const;

  void write(const TraceEvent& event);
  void write(TraceEventType type,
             nlohmann::json payload = nlohmann::json::object(),
             std::string instance_id = {},
             double wall_time_sec = 0.0);
  void close();

 private:
  void write_header();

  std::filesystem::path path_;
  TraceWriterOptions options_;
  std::ofstream stream_;
  std::uint64_t next_sequence_{1};
  std::uint64_t events_written_{0};
};

}  // namespace mipsolvers::l2o