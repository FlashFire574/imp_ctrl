"""App entry points."""

from .left_arm import main as left_arm_main
from .run_lab import main as run_lab_main

__all__ = ["left_arm_main", "run_lab_main"]
