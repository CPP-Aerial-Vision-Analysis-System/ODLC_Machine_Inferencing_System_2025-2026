import json
from pathlib import Path

STATE_FILE = Path(__file__).resolve().parent / "mission_state.json"

DEFAULTS = {
    "last_command": None,
    "timestamp": None,
    "last_sender_sysid": None,
    "last_sender_compid": None,
    "pending_action": None,
   

}

def load_state() -> dict:
    try:
        if STATE_FILE.exists():
            loaded = json.loads(STATE_FILE.read_text())
            return _merge_dicts(DEFAULTS, loaded)
    except Exception:
        pass
    return json.loads(json.dumps(DEFAULTS))

def update_state(key: str, value) -> None:
    state = load_state()
    state[key] = value
    STATE_FILE.write_text(json.dumps(state, indent=2))

def _merge_dicts(default: dict, override: dict) -> dict:
    result = dict(default)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(result.get(k), dict):
            result[k] = _merge_dicts(result[k], v)
        else:
            result[k] = v
    return result