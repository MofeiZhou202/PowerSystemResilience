#include "hacdcpf/io/roundtrip.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <string>

#include "hacdcpf/io/json_io.hpp"

namespace hacdcpf::io {
namespace {

using json = nlohmann::json;

// Maximum number of individual mismatches retained (a broken round-trip should
// not produce an unbounded evidence blob).
constexpr std::size_t kMaxMismatches = 64;

std::string scalar_to_string(const json& v) {
  if (v.is_string()) return v.get<std::string>();
  if (v.is_boolean()) return v.get<bool>() ? "true" : "false";
  if (v.is_null()) return "null";
  return v.dump();
}

void record_mismatch(RoundTripEvidence& ev, const std::string& path,
                     const json& a, const json& b) {
  ++ev.fields_mismatched;
  if (ev.mismatches.size() < kMaxMismatches) {
    ev.mismatches.push_back({path, scalar_to_string(a), scalar_to_string(b)});
  }
}

void diff_json(const json& a, const json& b, const std::string& path,
               double epsilon, RoundTripEvidence& ev) {
  if (a.is_number() && b.is_number()) {
    ++ev.fields_checked;
    const double da = a.get<double>();
    const double db = b.get<double>();
    const double tol = epsilon * std::max(1.0, std::abs(da));
    if (!(std::abs(da - db) <= tol)) record_mismatch(ev, path, a, b);
    return;
  }
  if (a.type() != b.type()) {
    ++ev.fields_checked;
    record_mismatch(ev, path, a, b);
    return;
  }
  if (a.is_object()) {
    // Union of keys so missing/extra fields are caught.
    for (auto it = a.begin(); it != a.end(); ++it) {
      const std::string child = path.empty() ? it.key() : path + "." + it.key();
      if (!b.contains(it.key())) {
        ++ev.fields_checked;
        record_mismatch(ev, child, it.value(), json(nullptr));
        continue;
      }
      diff_json(it.value(), b.at(it.key()), child, epsilon, ev);
    }
    for (auto it = b.begin(); it != b.end(); ++it) {
      if (!a.contains(it.key())) {
        const std::string child =
            path.empty() ? it.key() : path + "." + it.key();
        ++ev.fields_checked;
        record_mismatch(ev, child, json(nullptr), it.value());
      }
    }
    return;
  }
  if (a.is_array()) {
    const std::size_t n = std::max(a.size(), b.size());
    for (std::size_t i = 0; i < n; ++i) {
      const std::string child = path + "[" + std::to_string(i) + "]";
      if (i >= a.size() || i >= b.size()) {
        ++ev.fields_checked;
        record_mismatch(ev, child, i < a.size() ? a[i] : json(nullptr),
                        i < b.size() ? b[i] : json(nullptr));
        continue;
      }
      diff_json(a[i], b[i], child, epsilon, ev);
    }
    return;
  }
  // Scalar (string / bool / null): exact match.
  ++ev.fields_checked;
  if (a != b) record_mismatch(ev, path, a, b);
}

}  // namespace

RoundTripEvidence diff_systems(const HybridPowerSystem& before,
                               const HybridPowerSystem& after, double epsilon,
                               std::string adapter, std::string level) {
  RoundTripEvidence ev;
  ev.adapter = std::move(adapter);
  ev.level = std::move(level);
  ev.epsilon = epsilon;
  const json a = json::parse(to_json(before));
  const json b = json::parse(to_json(after));
  diff_json(a, b, "", epsilon, ev);
  ev.lossless = ev.fields_mismatched == 0;
  ev.passed = ev.lossless;
  return ev;
}

RoundTripEvidence json_roundtrip(const HybridPowerSystem& sys, double epsilon) {
  RoundTripEvidence ev;
  try {
    const HybridPowerSystem restored = from_json(to_json(sys));
    ev = diff_systems(sys, restored, epsilon, "json", "rich");
  } catch (const std::exception& e) {
    ev.adapter = "json";
    ev.level = "rich";
    ev.epsilon = epsilon;
    ev.lossless = false;
    ev.passed = false;
    ev.mismatches.push_back({"<exception>", "", e.what()});
  }
  return ev;
}

}  // namespace hacdcpf::io
