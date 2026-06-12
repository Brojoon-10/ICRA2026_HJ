# Repository cleanup & deployment guide

This repository is being cleaned up for external release, then published to
another remote. Work happens on branch `release/cleanup-and-deploy` (branched
from `main`; **never commit directly to `main`**). The goal is a codebase with
no personal traces and no internal-only clutter, while preserving everything
that is functionally meaningful.

## Scope — which files we touch

**Only files modified in 2026 or later are in scope.** This repo is built on an
upstream stack; files last changed in 2025 or earlier are upstream originals — do
not clean, rewrite, or delete them, even if they contain dead code, terse
comments, or non-English text. Our cleanup applies solely to what *we* added or
changed this year. (This is why e.g. `controller/combined/src/helpers.py` and
`plotter.py`, last touched in 2025, are left untouched.)

Check a file's scope before editing:

```
git log -1 --format='%ci' -- <file>   # 2026+ → in scope; 2025 or earlier → skip
```

Work proceeds **one package at a time, one file at a time**, leaving nothing
out within scope. Every session must follow the rules below identically.

## Absolute rules (set by the maintainer — do not break)

1. **Remove name/date traces.** Markers such as `### HJ :`, `HJ MODIFIED`,
   `HJ ADDED`, `HJ END`, `IY`, dated tags (`2026-05-25`, `0329`, `0404`, ...),
   `Emergency Editing`, etc.
   - Marker-only lines (e.g. `# ===== HJ MODIFIED END =====`) → **delete the line**.
   - Markers with a meaningful explanation (e.g. `### HJ : z coordinate for ...`)
     → **strip only the name/date tag, keep the explanation as a plain `#` comment**.

2. **Do not shrink or alter meaningful comments.** If a comment carries real
   information, keep it. The one exception: if a comment is **factually wrong**,
   correct it based on the code.

3. **Remove "insider-only" cross references.** Comments like
   `same metric as live_monitor` or `matches live_monitor's slice` — references
   to other files/context that mean nothing in a standalone release — delete them.

4. **All Korean → English.** Docstrings, comments, READMEs — everything.
   Also convert non-ASCII box-drawing characters (─ │ ┌ ┘ ...) in comments to ASCII.

5. **Delete dead code.** Commented-out alternative implementations, unused
   buffers/functions/imports. Verify a symbol is truly unused across the
   workspace before deleting.

6. **Fix docstrings/comments that don't match the code** (e.g. listing topics or
   paths that don't exist), using the code as the source of truth.

7. **No hardcoded absolute paths in scripts.** Derive paths dynamically; pull
   genuinely environment-specific values (container name, in-container workspace
   root) into overridable variables at the top of the file.

8. **Keep multi-file definitions in sync.** When a parameter/config is defined in
   one place and consumed in others, update all of them together and verify no
   orphans remain on either side.

9. **Ask before high-impact changes.** Deleting whole functions, moving/renaming
   files, restructuring, or removing a feature end-to-end → ask the maintainer
   first (offer clear options). Simple marker/Korean/dead-comment cleanup → just do it.

10. **Removing a feature = remove every trace of it.** A feature is rarely in one
    file. Before deleting, grep the whole repo for its symbols/file names to find
    *all* related definitions — code, `cfg/*.cfg`, `*.yaml`, `CMakeLists.txt`,
    `*.launch`, folders — and remove them together, or the build breaks / dead
    config ships. Then **confirm with the maintainer right before the actual
    deletion** ("found it in these N files, deleting now — ok?"), since a removal
    is hard to undo. Re-grep afterward to prove zero references remain.

## Per-file workflow

1. Scan: `grep -nP 'HJ|IY|MODIFIED|ADDED|[가-힣]|live_monitor' <file>`
2. Apply the rules above.
3. Validate syntax — Python: `python3 -c "import ast; ast.parse(open('<file>').read())"`;
   Bash: `bash -n <file>`.
4. Re-scan to confirm no traces remain.

## Per-package scan (run at the start and end of each package)

```
grep -rnP 'HJ|IY|MODIFIED|ADDED|Emergency Editing|[가-힣]|live_monitor' \
  <pkg>/ --include=*.py --include=*.cpp --include=*.h --include=*.hpp \
  --include=*.cfg --include=*.launch --include=*.xml --include=*.yaml \
  --include=*.md --include=*.txt --include=*.sh --include=CMakeLists.txt
```

## Commits

- Commit only when the maintainer says so. One logical change per commit;
  separate genuine bug fixes from cleanup so the history stays clear.
- **Do not add any `Co-Authored-By` or "Generated with" footer.** Commit messages
  must contain no AI/tooling trace — this is a deployment repo.

## README style (maintainer's preference)

- Concise and factual — no rambling. Introduce the main feature plus the
  available options, based on what the code actually does.
- Describe each option's **purpose**, not tuned numbers. For tuning-sensitive
  corrections, lean toward recommending the defaults (off / none).
