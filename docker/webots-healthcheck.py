import json
import os
import time
from pathlib import Path

state = Path(os.environ["WEB_STATE_DIR"]) / "robot_state.json"
started = Path("/tmp/webots-started").stat().st_mtime
observed = state.stat().st_mtime
if observed < started or time.time() - observed > 15:
    raise SystemExit(1)
with state.open() as source:
    telemetry = json.load(source)
if not telemetry.get("pose") or telemetry.get("navigation", {}).get("status") == "error":
    raise SystemExit(1)
