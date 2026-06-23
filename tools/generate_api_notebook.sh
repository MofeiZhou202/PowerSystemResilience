#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if ! command -v doxygen >/dev/null 2>&1; then
  echo "error: doxygen is not installed." >&2
  echo "Install it with: brew install doxygen graphviz" >&2
  exit 127
fi

mkdir -p docs/generated
doxygen Doxyfile

echo
echo "Generated HTML notebook:"
echo "  $repo_root/docs/generated/api/index.html"
echo "Doxygen warnings, if any:"
echo "  $repo_root/docs/generated/doxygen-warnings.log"
