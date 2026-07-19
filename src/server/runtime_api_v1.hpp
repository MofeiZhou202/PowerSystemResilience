#pragma once

#include <functional>
#include <memory>
#include <string>

#include <httplib.h>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::server {

class RuntimeApiV1 {
 public:
  using CaseBuilder =
      std::function<HybridPowerSystem(const std::string& case_name)>;

  explicit RuntimeApiV1(CaseBuilder case_builder, int worker_count = 2);
  ~RuntimeApiV1();

  RuntimeApiV1(const RuntimeApiV1&) = delete;
  RuntimeApiV1& operator=(const RuntimeApiV1&) = delete;

  void register_routes(httplib::Server& server);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hacdcpf::server

