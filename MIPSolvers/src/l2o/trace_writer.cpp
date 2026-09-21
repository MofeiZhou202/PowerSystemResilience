#include "mipsolvers/l2o/trace_writer.hpp"

#include <stdexcept>
#include <utility>

namespace mipsolvers::l2o {

TraceWriter::TraceWriter(const std::filesystem::path& path,
                         TraceWriterOptions options)
    : path_(path), options_(std::move(options)) {
  const auto parent = path_.parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);

  auto mode = std::ios::out;
  mode |= options_.append ? std::ios::app : std::ios::trunc;
  stream_.open(path_, mode);
  if (!stream_) {
    throw std::runtime_error("failed to open L2O trace file: " + path_.string());
  }
  if (options_.write_header) write_header();
}

TraceWriter::~TraceWriter() { close(); }

bool TraceWriter::is_open() const { return stream_.is_open(); }

const std::filesystem::path& TraceWriter::path() const { return path_; }

std::uint64_t TraceWriter::events_written() const { return events_written_; }

void TraceWriter::write(const TraceEvent& event) {
  if (!stream_) {
    throw std::runtime_error("cannot write to closed L2O trace file: " + path_.string());
  }

  TraceEvent out = event;
  if (out.sequence == 0 && out.type != TraceEventType::TraceHeader) {
    out.sequence = next_sequence_++;
  } else if (out.sequence >= next_sequence_) {
    next_sequence_ = out.sequence + 1;
  }
  if (out.run_id.empty()) out.run_id = options_.run_id;
  if (!out.payload.is_object()) {
    out.payload = nlohmann::json{{"value", out.payload}};
  }

  stream_ << nlohmann::json(out).dump() << '\n';
  if (!stream_) {
    throw std::runtime_error("failed while writing L2O trace file: " + path_.string());
  }
  ++events_written_;
  if (options_.flush_each_event) stream_.flush();
}

void TraceWriter::write(TraceEventType type,
                        nlohmann::json payload,
                        std::string instance_id,
                        double wall_time_sec) {
  write(make_trace_event(type, std::move(payload), std::move(instance_id), wall_time_sec));
}

void TraceWriter::close() {
  if (stream_.is_open()) stream_.close();
}

void TraceWriter::write_header() {
  TraceEvent header;
  header.type = TraceEventType::TraceHeader;
  header.sequence = 0;
  header.run_id = options_.run_id;
  header.payload = nlohmann::json{
      {"trace_schema_version", options_.trace_schema_version},
      {"metadata", options_.metadata}};
  write(header);
}

}  // namespace mipsolvers::l2o