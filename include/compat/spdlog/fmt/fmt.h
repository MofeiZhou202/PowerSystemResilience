// Compatibility shim: spdlog/fmt/fmt.h → fmt/core.h
// spdlog bundles fmt and exposes it via this header; since we use fmt directly
// we just forward to fmt's own headers.
#pragma once
#include <fmt/core.h>
#include <fmt/format.h>
