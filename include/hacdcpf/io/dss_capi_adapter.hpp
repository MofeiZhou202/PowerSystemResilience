#pragma once

// Project-level adapter for the vendored DSS C-API package.
// The current release bundle exposes the ctx_* API as altdss/capi/dss_ctx.h.
// Older dss_capi packages exposed the same API as dss_capi_ctx.h.
#if __has_include(<dss_capi_ctx.h>)
#include <dss_capi_ctx.h>
#elif __has_include(<altdss/capi/dss_ctx.h>)
#include <altdss/capi/dss_ctx.h>
#else
#error "DSS C-API header not found. Expected vendored third_party/dss_capi/dss_capi."
#endif
