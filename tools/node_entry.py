"""Start our own node directly, without ros2 run, colcon, or package registration."""
import importlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
NODES = {
    'gateway': 'gateway', 'mission_node': 'mission_node', 'preflight': 'preflight',
    'probe_topics': 'probe_topics', 'calibrate_drive': 'calibrate_drive',
    'record_node': 'record_node', 'speak_test': 'tts',
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in NODES:
        raise SystemExit('node must be one of: ' + ', '.join(NODES))
    name = NODES[sys.argv.pop(1)]
    sys.path[:0] = [str(ROOT), str(ROOT / 'ros2_ws/src/racecar_mission')]
    module = importlib.import_module('racecar_mission.' + name)
    if not Path(module.__file__).resolve().is_relative_to(ROOT):
        raise SystemExit('Refusing node imported from another installation')
    return module.main()


if __name__ == '__main__':
    sys.exit(main())
