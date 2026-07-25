#!/usr/bin/env bash
# third_party/build_third_party.sh
# Builds and installs the prebuilt third-party package for MIPSolvers:
# configures the project with -DMIPSOLVERS_THIRD_PARTY_ONLY=ON, builds every
# vendored dependency (SuiteSparse/CHOLMOD, MUMPS, HiGHS, SCIP, Ipopt, fmt,
# Catch2, and reference LAPACK when the BLAS fallback triggers), and installs
# libs + headers + the mipsolversThirdPartyTargets export + package config +
# manifest.cmake into third_party/install.
#
# Day-to-day builds then pick the package up automatically
# (MIPSOLVERS_USE_PREBUILT_THIRD_PARTY=AUTO) and skip recompiling the
# vendored translation units.
#
# Usage: third_party/build_third_party.sh [--jobs N] [--prefix P]
#                                         [--build-type T] [--fresh]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/third_party/build"

if command -v sysctl >/dev/null 2>&1 &&
   DEFAULT_JOBS="$(sysctl -n hw.ncpu 2>/dev/null)"; then
  :
elif command -v nproc >/dev/null 2>&1; then
  DEFAULT_JOBS="$(nproc)"
else
  DEFAULT_JOBS=4
fi

JOBS="${DEFAULT_JOBS}"
PREFIX="${REPO_ROOT}/third_party/install"
BUILD_TYPE="Release"
FRESH=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --jobs)
      JOBS="${2:?--jobs requires a value}"; shift 2 ;;
    --jobs=*)
      JOBS="${1#*=}"; shift ;;
    --prefix)
      PREFIX="${2:?--prefix requires a value}"; shift 2 ;;
    --prefix=*)
      PREFIX="${1#*=}"; shift ;;
    --build-type)
      BUILD_TYPE="${2:?--build-type requires a value}"; shift 2 ;;
    --build-type=*)
      BUILD_TYPE="${1#*=}"; shift ;;
    --fresh)
      FRESH=1; shift ;;
    -h|--help)
      grep '^#' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' | head -n 18
      exit 0 ;;
    *)
      echo "error: unknown option '$1' (see --help)" >&2
      exit 2 ;;
  esac
done

if ! [[ "${JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "error: --jobs must be a positive integer (got '${JOBS}')" >&2
  exit 2
fi
if [[ -z "${PREFIX}" ]]; then
  echo "error: --prefix must not be empty" >&2
  exit 2
fi
if [[ "${PREFIX}" != /* ]]; then
  PREFIX="${PWD}/${PREFIX}"
fi
if ! command -v cmake >/dev/null 2>&1; then
  echo "error: cmake is required but was not found in PATH" >&2
  exit 127
fi

if [[ ${FRESH} -eq 1 ]]; then
  echo ">> wiping ${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
fi

START_TIME="${SECONDS}"

echo ">> configuring third-party-only build (${BUILD_TYPE}) in ${BUILD_DIR}"
cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" \
  -DMIPSOLVERS_THIRD_PARTY_ONLY=ON \
  -DMIPSOLVERS_THIRD_PARTY_BUILD_CONFIG="${BUILD_TYPE}" \
  -DCMAKE_CONFIGURATION_TYPES="${BUILD_TYPE}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}"

echo ">> building vendored third-party libraries with ${JOBS} jobs"
cmake --build "${BUILD_DIR}" --config "${BUILD_TYPE}" --parallel "${JOBS}"

echo ">> installing to ${PREFIX}"
cmake --install "${BUILD_DIR}" --config "${BUILD_TYPE}"

ELAPSED=$(( SECONDS - START_TIME ))

echo ""
echo "== prebuilt third-party summary =="
echo "prefix:     ${PREFIX}"
echo "build type: ${BUILD_TYPE}"
echo "wall time:  $(( ELAPSED / 60 ))m $(( ELAPSED % 60 ))s"
if [[ -d "${PREFIX}/lib" ]]; then
  echo "libraries installed:"
  find "${PREFIX}/lib" -maxdepth 1 \( -name '*.a' -o -name '*.dylib' -o \
    -name '*.so' -o -name '*.lib' -o -name '*.dll' -o -name '*.dll.a' \) \
    -exec du -h {} + | sort -k2
fi
PACKAGE_DIR=""
for CANDIDATE in \
  "${PREFIX}/lib/cmake/mipsolvers-third-party" \
  "${PREFIX}/lib64/cmake/mipsolvers-third-party" \
  "${PREFIX}/share/mipsolvers-third-party"; do
  if [[ -f "${CANDIDATE}/mipsolversThirdPartyConfig.cmake" ]]; then
    PACKAGE_DIR="${CANDIDATE}"
    break
  fi
done
if [[ -n "${PACKAGE_DIR}" ]]; then
  echo "package config: ${PACKAGE_DIR}/mipsolversThirdPartyConfig.cmake"
  echo "manifest:       ${PACKAGE_DIR}/manifest.cmake"
else
  echo "warning: installed package config was not found under ${PREFIX}" >&2
fi
