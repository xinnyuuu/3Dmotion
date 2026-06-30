# Example Sessions

This directory contains small tracked excerpts that are safe to keep in git.

## `session_20260630_141240_excerpt`

Source: `data/raw/session_20260630_141240`

Purpose: reproducible smoke test for the current AprilGrid world-anchor MVP.

Contents:

```text
cameras/frames.jsonl
cameras/C0..C3/*.jpg
imus/head_imu.jsonl
imus/wrist_imu.jsonl
session_manifest.json
```

Run from the repository root:

```bash
source .venv/bin/activate
python scripts/process_dashboard_session.py \
  data/examples/session_20260630_141240_excerpt \
  --hands
```

Outputs are written under:

```text
data/processed/session_20260630_141240_excerpt/
```

`data/raw/` and `data/processed/` are intentionally ignored by git. Keep only compact examples in this directory.
