#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from packages.head_vio_bridge.p3_head_vio import readiness_main


if __name__ == "__main__":
    readiness_main()

