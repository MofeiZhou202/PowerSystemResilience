#!/usr/bin/env python3
"""One-shot migration: convert `file.cpp:LINE` doc anchors to `file.cpp:symbol`.

Handles three anchor forms inside docs/modules/**/*.tex and docs/{theory,
developer,overview,reference,guides}/**/*.md:

  full      path/to/foo.cpp:123  or  foo.cpp:123--456   (tex may escape `_`)
  samefile  同文件:123(--456)
  bare      （:123） / (:123) / （:123--456）

The symbol at LINE is the nearest preceding function/struct/class/enum
definition in the referenced source file.  Unresolved anchors keep the file
part and drop the line number, and are listed in the report for manual review.

Usage: python3 tools/doc_anchor_symbolize.py [--apply]
"""
import os, re, sys, glob, json
from collections import defaultdict

APPLY = '--apply' in sys.argv

DOC_GLOBS = ['docs/modules/**/*.tex',
             'docs/theory/**/*.md', 'docs/developer/**/*.md',
             'docs/overview/**/*.md', 'docs/reference/**/*.md',
             'docs/guides/**/*.md']
SKIP_SUBSTR = ('/archive/', '/latex/')

# ---------- source file index (basename -> [paths]) ----------
SRC_INDEX = defaultdict(list)
for base in ('src', 'include', 'tests'):
    for root, dirs, files in os.walk(base):
        if 'node_modules' in root: continue
        for f in files:
            if f.endswith(('.cpp', '.hpp', '.h')):
                SRC_INDEX[f].append(os.path.join(root, f))

# Legacy basenames that no longer exist (renamed files), mapped to the current
# source path.  Keyed by basename after unescaping.
RENAME_MAP = {
    'opf.cpp': 'src/optimal_power_flow/three_phase_hybrid_opf.cpp',
    'adapter.cpp': 'src/optimal_power_flow/three_phase_hybrid_adapter.cpp',
    'relaxation.cpp': 'src/optimal_power_flow/three_phase_hybrid_relaxation.cpp',
    'relaxation.hpp': 'include/hacdcpf/optimal_power_flow/three_phase_hybrid_relaxation.hpp',
}

def resolve_source(name, doc_path):
    name = name.replace('\\_', '_')
    if os.path.exists(name):
        return name
    base = os.path.basename(name)
    if base in RENAME_MAP and os.path.exists(RENAME_MAP[base]):
        return RENAME_MAP[base]
    cands = SRC_INDEX.get(base, [])
    if not cands:
        return None
    if len(cands) == 1:
        return cands[0]
    # prefer path sharing the module dir name with the doc
    m = re.search(r'docs/modules/([^/]+)/', doc_path)
    if m:
        mod = m.group(1)
        for c in cands:
            if f'/{mod}/' in c:
                return c
    for c in cands:  # prefer src/ then include/ over tests/
        if c.startswith('src/'): return c
    return cands[0]

# ---------- symbol table per source file ----------
CONTROL = {'if', 'for', 'while', 'switch', 'catch', 'return', 'sizeof', 'else', 'do'}
LOOKAHEAD = 30

_TYPE_HEAD = re.compile(
    r'^\s*(?:template\s*<[^>]*>\s*)?(?:static\s+|inline\s+|constexpr\s+|virtual\s+|explicit\s+|friend\s+|extern\s+)*'
    r'(?:[A-Za-z_][\w:<>,&*\[\]\s~]*?\s+)?([A-Za-z_]\w*(?:::[A-Za-z_]\w*)*|~[A-Za-z_]\w*|operator[^\s(]*)\s*\(')
_TYPE_KIND = re.compile(r'^\s*(?:template\s*<[^>]*>\s*)?(struct|class|enum(?:\s+class)?)\s+([A-Za-z_]\w*)')
_LAMBDA = re.compile(r'^\s*(?:const\s+)?auto\s+([A-Za-z_]\w*)\s*=\s*\[[^\]]*\]\s*[({]?')

def _close_paren_follows(lines, li, col):
    """From lines[li][col] (the '('), scan a balanced paren group across lines;
    return the first significant text after the matching ')'."""
    depth = 0
    i, j = li, col
    while i < len(lines):
        line = lines[i]
        while j < len(line):
            ch = line[j]
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    rest = line[j + 1:]
                    k = i + 1
                    while rest.strip() == '' and k < len(lines) and k < i + 8:
                        rest = lines[k]; k += 1
                    return rest.strip()
            j += 1
        i += 1
        j = 0
        if i > li + 40:  # long multi-line signatures
            break
    return None

_sym_cache = {}
def symbol_at(src_path, line_no):
    if src_path not in _sym_cache:
        is_header = src_path.endswith(('.hpp', '.h'))
        defs = []
        try:
            lines = open(src_path, encoding='utf-8', errors='ignore').read().splitlines()
        except OSError:
            lines = []
        for i, l in enumerate(lines):
            s = l.strip()
            if not s or s.startswith('//') or s.startswith('*') or s.startswith('#'):
                continue
            m = _TYPE_KIND.match(l)
            if m:
                defs.append((i + 1, m.group(2))); continue
            if 'throw' in l or 'return' in l:
                continue
            m = _LAMBDA.match(l)
            if m:
                defs.append((i + 1, m.group(1) + '（lambda）')); continue
            m = _TYPE_HEAD.match(l)
            if not m:
                continue
            name = m.group(1).split('::')[-1].lstrip('~')
            if name in CONTROL:
                continue
            head = l[:m.start(1)]
            if '=' in head or head.rstrip().endswith(('.', '->')):
                continue  # assignment/call expression, not a definition
            follow = _close_paren_follows(lines, i, m.end(0) - 1)
            if follow is None:
                continue
            if follow.startswith('{') or follow.startswith(':') or \
               re.match(r'^(const|noexcept|override|final|requires|->|\btry\b)[^{;]*\{', follow):
                defs.append((i + 1, name))
            elif follow.startswith(';') and is_header:
                defs.append((i + 1, name))  # function declaration in a header
        _sym_cache[src_path] = defs
    defs = _sym_cache[src_path]
    best = None
    for start, name in defs:
        if start <= line_no:
            best = name
        else:
            break
    if best is None and defs and defs[0][0] <= line_no + LOOKAHEAD:
        best = defs[0][1]  # anchor region sits just before/around the first def
    return best

# ---------- anchor rewriting ----------
FULL = re.compile(r'((?:[A-Za-z0-9_./\\-]+/)?[A-Za-z0-9_\\.-]+\.(?:cpp|hpp|h)):(\d+)(?:--(\d+))?')
SAME = re.compile(r'同文件:(\d+)(?:--(\d+))?')
BARE = re.compile(r'([（(])((?::\d+(?:--\d+)?\s*[，,、]?\s*)+)([）)])')

report = {'resolved': 0, 'unresolved': [], 'files_changed': 0}

def esc(name, tex):
    return name.replace('_', '\\_') if tex else name

def process(doc_path):
    tex = doc_path.endswith('.tex')
    text = open(doc_path, encoding='utf-8', errors='ignore').read()
    state = {'last_src': None}
    local_unresolved = []

    def full_repl(m):
        ref, start = m.group(1), int(m.group(2))
        src = resolve_source(ref, doc_path)
        if not src:
            local_unresolved.append(('no-source', m.group(0))); return m.group(0)
        state['last_src'] = src
        sym = symbol_at(src, start)
        # if the ref itself is stale (renamed/moved file), point the display at
        # the resolved current path, preserving tex escaping
        disp = ref
        if not os.path.exists(ref) and not os.path.exists(ref.replace('\\_', '_')):
            disp = esc(src, tex) if tex else src
        if sym:
            report['resolved'] += 1
            return f"{disp}:{esc(sym, tex)}"
        local_unresolved.append(('no-symbol', m.group(0), src))
        return disp

    def same_repl(m):
        if not state['last_src']:
            local_unresolved.append(('no-context', m.group(0))); return m.group(0)
        sym = symbol_at(state['last_src'], int(m.group(1)))
        if sym:
            report['resolved'] += 1
            return f"同文件:{esc(sym, tex)}"
        local_unresolved.append(('no-symbol', m.group(0), state['last_src']))
        return '同文件'

    def bare_repl(m):
        if not state['last_src']:
            local_unresolved.append(('no-context', m.group(0))); return m.group(0)
        open_c, body, close_c = m.group(1), m.group(2), m.group(3)
        anchors = re.findall(r':(\d+)(?:--\d+)?', body)
        if len(anchors) > 1:
            # multi-entry lists too often rely on prose context that
            # last-src tracking cannot recover reliably — leave for manual fix
            local_unresolved.append(('manual-review-list', m.group(0), state['last_src']))
            return m.group(0)
        sym = symbol_at(state['last_src'], int(anchors[0]))
        if sym:
            report['resolved'] += 1
            return f"{open_c}:{esc(sym, tex)}{close_c}"
        local_unresolved.append(('no-symbol', m.group(0), state['last_src']))
        return open_c + close_c

    # per-line so "last seen file" tracking follows reading order
    NAME = re.compile(r'(?:[A-Za-z0-9_./\\-]+/)?[A-Za-z0-9_\\-]+\.(?:cpp|hpp|h)')
    out_lines = []
    for line in text.splitlines():
        # plain filename mentions (prose like \fld{ac\_opf.cpp}) also set context
        names = NAME.findall(line)
        if names:
            src = resolve_source(names[-1], doc_path)
            if src:
                state['last_src'] = src
        line = BARE.sub(bare_repl, line)
        line = SAME.sub(same_repl, line)
        line = FULL.sub(full_repl, line)
        out_lines.append(line)
    new = '\n'.join(out_lines) + ('\n' if text.endswith('\n') else '')
    if new != text:
        report['files_changed'] += 1
        if APPLY:
            open(doc_path, 'w', encoding='utf-8').write(new)
    if local_unresolved:
        report['unresolved'].append((doc_path, local_unresolved))

for pat in DOC_GLOBS:
    for p in sorted(glob.glob(pat, recursive=True)):
        if any(s in p for s in SKIP_SUBSTR):
            continue
        process(p)

n_unres = sum(len(x[1]) for x in report['unresolved'])
print(f"mode={'APPLY' if APPLY else 'DRY-RUN'} resolved={report['resolved']} "
      f"unresolved={n_unres} files_changed={report['files_changed']}")
for doc, items in report['unresolved']:
    print(f"  {doc}")
    for it in items[:8]:
        print('   ', it)
    if len(items) > 8:
        print(f"    ... and {len(items)-8} more")
