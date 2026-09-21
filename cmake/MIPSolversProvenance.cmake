# Validate the vendored MIPSolvers source against the reviewed import lock.
# The full Git tree is checked by release tooling once the prefixed import is
# committed. CMake also verifies stable files so source archives without Git
# metadata cannot silently select an unrelated dependency revision.

set(_HACDCDSS_MIPSOLVERS_LOCK_FILE
    "${CMAKE_CURRENT_LIST_DIR}/MIPSolvers.lock.json")
if(NOT EXISTS "${_HACDCDSS_MIPSOLVERS_LOCK_FILE}")
  message(FATAL_ERROR
    "Missing MIPSolvers provenance lock: ${_HACDCDSS_MIPSOLVERS_LOCK_FILE}")
endif()

file(READ "${_HACDCDSS_MIPSOLVERS_LOCK_FILE}"
     _HACDCDSS_MIPSOLVERS_LOCK_JSON)
string(JSON _HACDCDSS_MIPSOLVERS_LOCK_SCHEMA ERROR_VARIABLE _lock_error
       GET "${_HACDCDSS_MIPSOLVERS_LOCK_JSON}" schema)
if(_lock_error OR
   NOT _HACDCDSS_MIPSOLVERS_LOCK_SCHEMA STREQUAL
       "hacdcpf.dependency-lock.v1")
  message(FATAL_ERROR
    "Invalid MIPSolvers provenance lock schema in "
    "${_HACDCDSS_MIPSOLVERS_LOCK_FILE}: ${_lock_error}")
endif()

string(JSON _HACDCDSS_MIPSOLVERS_LOCK_NAME
       GET "${_HACDCDSS_MIPSOLVERS_LOCK_JSON}" dependency)
if(NOT _HACDCDSS_MIPSOLVERS_LOCK_NAME STREQUAL "MIPSolvers")
  message(FATAL_ERROR
    "The dependency lock names '${_HACDCDSS_MIPSOLVERS_LOCK_NAME}', expected "
    "'MIPSolvers'.")
endif()

string(JSON _HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT
       GET "${_HACDCDSS_MIPSOLVERS_LOCK_JSON}" upstream commit)
string(JSON _HACDCDSS_MIPSOLVERS_EXPECTED_TREE
       GET "${_HACDCDSS_MIPSOLVERS_LOCK_JSON}" import git_tree)
set(_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT
    "${_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT}" CACHE STRING
    "Expected MIPSolvers upstream commit from the provenance lock" FORCE)
set(_HACDCDSS_MIPSOLVERS_EXPECTED_TREE
    "${_HACDCDSS_MIPSOLVERS_EXPECTED_TREE}" CACHE STRING
    "Expected Git tree of the prefixed MIPSolvers import" FORCE)

function(_hacdcpf_verify_mipsolvers_lock source_dir)
  string(JSON _verification_count LENGTH
         "${_HACDCDSS_MIPSOLVERS_LOCK_JSON}" verification_files)
  if(_verification_count EQUAL 0)
    message(FATAL_ERROR
      "MIPSolvers provenance lock contains no verification files.")
  endif()

  math(EXPR _last_verification_index "${_verification_count} - 1")
  foreach(_index RANGE 0 ${_last_verification_index})
    string(JSON _relative_path GET
           "${_HACDCDSS_MIPSOLVERS_LOCK_JSON}"
           verification_files ${_index} path)
    string(JSON _expected_sha256 GET
           "${_HACDCDSS_MIPSOLVERS_LOCK_JSON}"
           verification_files ${_index} sha256)

    if(IS_ABSOLUTE "${_relative_path}" OR
       _relative_path MATCHES "(^|[/\\\\])\\.\\.([/\\\\]|$)")
      message(FATAL_ERROR
        "Unsafe path in MIPSolvers provenance lock: ${_relative_path}")
    endif()

    set(_candidate "${source_dir}/${_relative_path}")
    if(NOT EXISTS "${_candidate}")
      message(FATAL_ERROR
        "MIPSolvers provenance file is missing: ${_candidate}")
    endif()
    file(SHA256 "${_candidate}" _actual_sha256)
    if(NOT _actual_sha256 STREQUAL _expected_sha256)
      message(FATAL_ERROR
        "MIPSolvers provenance mismatch for ${_relative_path}.\n"
        "Expected SHA-256: ${_expected_sha256}\n"
        "Actual SHA-256:   ${_actual_sha256}\n"
        "Use the locked MIPSolvers import or intentionally update the lock.")
    endif()
  endforeach()

  message(STATUS
    "hacdcdss: MIPSolvers provenance files match commit "
    "${_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT}")
endfunction()
