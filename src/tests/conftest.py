# let tests import the modules in src/ directly (import kinematics, import config)
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
