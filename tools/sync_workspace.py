"""Local staging only; never connects to a car."""
from pathlib import Path
import shutil
root=Path(__file__).resolve().parents[1]
dst=root/'ros2_ws/src/racelib/racelib'
dst.mkdir(parents=True,exist_ok=True)
for p in (root/'racelib').glob('*.py'):shutil.copy2(p,dst/p.name)
native_dst=dst/'native'
native_dst.mkdir(parents=True,exist_ok=True)
for p in (root/'racelib/native').glob('*'):
    if p.is_file() and p.suffix == '.cpp':shutil.copy2(p,native_dst/p.name)
for p in (root/'config').glob('*.json'):shutil.copy2(p,root/'ros2_ws/src/racecar_mission/config'/p.name)

# Machine-specific binaries are built and loaded explicitly in the target runtime.
for p in native_dst.glob("*.so"): p.unlink()
