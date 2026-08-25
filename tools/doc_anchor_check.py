#!/usr/bin/env python3
"""Validate documentation source anchors against the current code base.

Every ``file:symbol`` anchor in the LaTeX manuals and Markdown design docs is a
claim that a named symbol still lives in a named source file.  The anchors are
authored either inside the ``\\srcpath{...}`` / ``\\implfull{...}`` macros and the
second argument of ``\\compmeta{name}{path}`` (see
``docs/_manual_common/hysim_manual.sty`` and ``theory_environments.tex``), or as
bare ``path.ext:symbol`` references in Markdown/prose.

This checker is the inverse of ``tools/doc_anchor_symbolize.py`` (which *creates*
the anchors): it fails when an anchor has drifted, i.e.

  * the referenced source file was renamed or removed, or
  * the referenced symbol no longer appears in the resolved file, or
  * a path-only anchor points at a non-existent file/directory.

It performs a *presence* check — the symbol's identifier appears as a whole word
in the resolved file's text — which is robust to source formatting and to the
parser limits of the symbolizer, while still catching real renames/removals.
Only ``file:symbol`` and structured path anchors can fail; bare identifiers
(``\\srcpath{solve}`` continuations, function names without a file) are counted
but never failed, because validating them needs prose context.

Because this checker also scans Markdown prose, changelogs/audit ledgers that
*discuss* a renamed or removed symbol must write the dead reference as
``symbol`` in ``file`` form (two separate spans) rather than the live
``file:symbol`` anchor syntax, so a fix write-up does not re-flag itself.

Usage::

    python3 tools/doc_anchor_check.py [--json] [--list-ok]

Exit status is 1 if any ``file:symbol`` or structured path anchor fails to
resolve, 0 otherwise.  Requires no build and no external dependency, so it runs
in the always-on CI lint job.
"""
from __future__ import annotations

import json
import os
import re
import sys
from collections import defaultdict

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Documentation trees to scan.  Archived snapshots and the LaTeX build scratch
# tree are intentionally excluded (they are not maintained contracts).
DOC_DIRS = ("docs",)
DOC_EXTS = (".tex", ".md")
SKIP_SUBSTR = (os.sep + "archive" + os.sep, os.sep + "latex" + os.sep)

SRC_EXT = (".cpp", ".hpp", ".h")

# A path-only anchor is only *failable* when it is rooted at one of these
# tracked source/build trees (present in every checkout) or is a bare source
# basename.  Anything else typeset inside \srcpath{} — HTTP routes
# ("POST /api/..."), slash-joined symbol lists ("is_connected/is_radial"),
# relative sub-directory fragments ("solvers/"), docs/ cross-references and
# environment-dependent data trees (external_data/, data/, output/) — is treated
# as a bare token and never fails.
VALIDATE_PREFIXES = ("src/", "include/", "tests/", "tools/", "web/", "cmake/")

# The sole external dependency is the sibling MIPSolvers repository (../MIPSolvers,
# outside this checkout); anchors into it cannot be resolved locally and are
# skipped rather than failed.
EXTERNAL_MARKERS = ("MIPSolvers/", "/MIPSolvers")

# Legacy basenames that no longer exist (renamed files), mapped to the current
# source path.  Kept in sync with tools/doc_anchor_symbolize.py.
RENAME_MAP = {
    "opf.cpp": "src/optimal_power_flow/three_phase_hybrid_opf.cpp",
    "adapter.cpp": "src/optimal_power_flow/three_phase_hybrid_adapter.cpp",
    "relaxation.cpp": "src/optimal_power_flow/three_phase_hybrid_relaxation.cpp",
    "relaxation.hpp": "include/hacdcpf/optimal_power_flow/three_phase_hybrid_relaxation.hpp",
}

# ---------------------------------------------------------------------------
# Source index: basename -> [absolute paths]; plus cached file text.
# ---------------------------------------------------------------------------
BASENAME_INDEX: dict[str, list[str]] = defaultdict(list)
for base in ("src", "include", "tests", "tools"):
    root_dir = os.path.join(REPO, base)
    for root, dirs, files in os.walk(root_dir):
        if "node_modules" in root:
            continue
        for f in files:
            if f.endswith(SRC_EXT):
                BASENAME_INDEX[f].append(os.path.join(root, f))

_text_cache: dict[str, str] = {}


def file_text(abs_path: str) -> str:
    t = _text_cache.get(abs_path)
    if t is None:
        try:
            with open(abs_path, encoding="utf-8", errors="ignore") as fh:
                t = fh.read()
        except OSError:
            t = ""
        _text_cache[abs_path] = t
    return t


def unescape_tex(s: str) -> str:
    for a, b in (("\\_", "_"), ("\\%", "%"), ("\\&", "&"), ("\\#", "#"),
                 ("\\{", "{"), ("\\}", "}"), ("\\$", "$")):
        s = s.replace(a, b)
    # Drop line-breaking spacing macros that authors embed inside long paths.
    s = re.sub(r"\\(allowbreak|-|,|;|:|/)\s*", "", s)
    return s


def _module_of(doc_rel: str) -> str | None:
    m = re.search(r"docs/modules/([^/]+)/", doc_rel.replace(os.sep, "/"))
    return m.group(1) if m else None


def resolve_source(fileref: str, doc_rel: str) -> str | None:
    """Resolve a doc file reference to an absolute source path, or None."""
    fileref = unescape_tex(fileref)
    direct = os.path.join(REPO, fileref)
    if os.path.isfile(direct):
        return direct
    base = os.path.basename(fileref)
    if base in RENAME_MAP:
        rp = os.path.join(REPO, RENAME_MAP[base])
        if os.path.isfile(rp):
            return rp
    cands = BASENAME_INDEX.get(base, [])
    if not cands:
        return None
    if len(cands) == 1:
        return cands[0]
    mod = _module_of(doc_rel)
    if mod:
        for c in cands:
            if f"{os.sep}{mod}{os.sep}" in c:
                return c
    for c in cands:  # prefer library source over tests/tools
        if (os.sep + "src" + os.sep) in c or (os.sep + "include" + os.sep) in c:
            return c
    return cands[0]


def path_exists(pathref: str) -> bool:
    """True if a path-only anchor (file or directory) resolves."""
    pathref = unescape_tex(pathref)
    # Dual-extension shorthand "foo.cpp/.hpp" names two sibling files at once;
    # the header commonly lives under include/ while the .cpp sits under src/,
    # so resolve each sibling through the basename index rather than literally.
    m = re.match(r"^(.*)\.(cpp|hpp|h)/\.(cpp|hpp|h)$", pathref)
    if m:
        base, e1, e2 = m.group(1), m.group(2), m.group(3)
        return (resolve_source(f"{base}.{e1}", "") is not None
                and resolve_source(f"{base}.{e2}", "") is not None)
    direct = os.path.join(REPO, pathref)
    if os.path.exists(direct):
        return True
    base = os.path.basename(pathref.rstrip("/"))
    if "/" not in pathref.rstrip("/") and base.endswith(SRC_EXT):
        return bool(resolve_source(pathref, ""))
    if base in RENAME_MAP:
        return os.path.exists(os.path.join(REPO, RENAME_MAP[base]))
    return False


_WORD = {}


def _word_present(text: str, ident: str) -> bool:
    rx = _WORD.get(ident)
    if rx is None:
        rx = re.compile(r"(?<![A-Za-z0-9_])" + re.escape(ident) + r"(?![A-Za-z0-9_])")
        _WORD[ident] = rx
    return bool(rx.search(text))


def symbol_present(text: str, symbol: str) -> bool:
    """Every identifier component of ``symbol`` appears as a word in ``text``.

    Handles ``Class::method`` (all components required), destructors (``~T``),
    ``operator...`` (leniently reduced to the ``operator`` keyword), trailing
    prose/lambda markers that the extraction may leave attached, and a trailing
    ``*`` wildcard that denotes a symbol family (``add_equivalent_branch_*``),
    which is matched as a prefix on the final component.
    """
    wildcard = symbol.endswith("*")
    if wildcard:
        symbol = symbol[:-1]
    comps = symbol.split("::")
    for idx, comp in enumerate(comps):
        m = re.match(r"~?([A-Za-z_]\w*)", comp.strip())
        if not m:
            continue
        ident = m.group(1)
        if ident == "operator":
            continue
        if wildcard and idx == len(comps) - 1:
            if not re.search(r"(?<![A-Za-z0-9_])" + re.escape(ident), text):
                return False
        elif not _word_present(text, ident):
            return False
    return True


def is_validatable_path(p: str) -> bool:
    """True only for path-only anchors whose existence can be trusted in any
    checkout: rooted at a tracked source tree, or a bare source basename."""
    p = p.strip()
    if not p or "*" in p or " " in p:  # globs and prose are not literal paths
        return False
    if any(m in p for m in EXTERNAL_MARKERS):
        return False
    if p.startswith(VALIDATE_PREFIXES):
        return True
    return "/" not in p.rstrip("/") and p.endswith(SRC_EXT)


# ---------------------------------------------------------------------------
# Anchor extraction.
# ---------------------------------------------------------------------------
# file:symbol, matched on tex-unescaped text so \_ becomes _ first.  Captures an
# optional directory prefix, a basename with a source extension, and a
# Class::method / symbol identifier chain.
FILESYM_RE = re.compile(
    r"(?<![A-Za-z0-9_])"
    r"((?:[\w./-]+/)?[\w.-]+\.(?:cpp|hpp|h))"
    r":"
    r"(~?[A-Za-z_]\w*(?:::~?[A-Za-z_]\w*)*\*?)"
)

SRCPATH_RE = re.compile(r"\\srcpath\{([^{}]*)\}")
IMPLFULL_RE = re.compile(r"\\implfull\{([^{}]*)\}")
COMPMETA_RE = re.compile(
    r"\\compmeta\{(?:[^{}]|\{[^{}]*\})*\}\{((?:[^{}]|\{[^{}]*\})*)\}"
)
UNWRAP_SRCPATH = re.compile(r"^\s*\\srcpath\{([^{}]*)\}\s*$")


class Report:
    def __init__(self) -> None:
        self.filesym_ok = 0
        self.path_ok = 0
        self.bare = 0
        self.failures: list[dict] = []

    def fail(self, kind: str, doc: str, line: int, anchor: str, reason: str) -> None:
        self.failures.append({"kind": kind, "doc": doc, "line": line,
                              "anchor": anchor, "reason": reason})


def check_filesym(report: Report, doc_rel: str, line: int, fileref: str,
                  symbol: str, seen: set) -> None:
    key = (doc_rel, line, fileref, symbol)
    if key in seen:
        return
    seen.add(key)
    if any(m in fileref for m in EXTERNAL_MARKERS):
        return  # external MIPSolvers sibling repo — not resolvable locally
    src = resolve_source(fileref, doc_rel)
    if src is None:
        report.fail("file", doc_rel, line, f"{fileref}:{symbol}",
                    "source file not found")
        return
    if not symbol_present(file_text(src), symbol):
        rel = os.path.relpath(src, REPO)
        report.fail("symbol", doc_rel, line, f"{fileref}:{symbol}",
                    f"symbol not found in {rel}")
        return
    report.filesym_ok += 1


def check_pathonly(report: Report, doc_rel: str, line: int, payload: str) -> None:
    p = unescape_tex(payload).strip()
    if not p or ":" in p:
        return  # file:symbol handled elsewhere
    if not is_validatable_path(p):
        report.bare += 1
        return
    if path_exists(p):
        report.path_ok += 1
    else:
        report.fail("path", doc_rel, line, p, "path does not exist")


def scan_doc(report: Report, abs_path: str) -> None:
    doc_rel = os.path.relpath(abs_path, REPO)
    seen: set = set()
    with open(abs_path, encoding="utf-8", errors="ignore") as fh:
        lines = fh.readlines()
    is_tex = abs_path.endswith(".tex")
    for i, raw in enumerate(lines, 1):
        # file:symbol anchors (structured or prose) on unescaped text
        text = unescape_tex(raw) if is_tex else raw
        for m in FILESYM_RE.finditer(text):
            check_filesym(report, doc_rel, i, m.group(1), m.group(2), seen)
        if not is_tex:
            continue
        # path-only anchors live only inside the structured macros
        for m in SRCPATH_RE.finditer(raw):
            check_pathonly(report, doc_rel, i, m.group(1))
        for m in IMPLFULL_RE.finditer(raw):
            check_pathonly(report, doc_rel, i, m.group(1))
        for m in COMPMETA_RE.finditer(raw):
            payload = m.group(1)
            um = UNWRAP_SRCPATH.match(payload)
            if um:
                payload = um.group(1)
            check_pathonly(report, doc_rel, i, payload)


def iter_docs():
    for d in DOC_DIRS:
        for root, dirs, files in os.walk(os.path.join(REPO, d)):
            if any(s in root + os.sep for s in SKIP_SUBSTR):
                continue
            for f in sorted(files):
                if f.endswith(DOC_EXTS):
                    yield os.path.join(root, f)


def main(argv: list[str]) -> int:
    as_json = "--json" in argv
    report = Report()
    for doc in iter_docs():
        if any(s in doc for s in SKIP_SUBSTR):
            continue
        scan_doc(report, doc)

    if as_json:
        print(json.dumps({
            "filesym_ok": report.filesym_ok,
            "path_ok": report.path_ok,
            "bare": report.bare,
            "failures": report.failures,
        }, ensure_ascii=False, indent=2))
    else:
        print(f"doc-anchor check: file:symbol ok={report.filesym_ok} "
              f"path ok={report.path_ok} bare={report.bare} "
              f"failures={len(report.failures)}")
        by_doc: dict[str, list[dict]] = defaultdict(list)
        for f in report.failures:
            by_doc[f["doc"]].append(f)
        for doc in sorted(by_doc):
            print(f"  {doc}")
            for f in by_doc[doc]:
                print(f"    L{f['line']} [{f['kind']}] {f['anchor']} — {f['reason']}")

    return 1 if report.failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
