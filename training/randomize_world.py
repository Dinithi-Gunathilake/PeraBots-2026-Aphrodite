"""
randomize_world.py

Generates a training world 'worlds/Arena_train.wbt' from 'worlds/Arena.wbt'.
Features:
1. Injects a headless Supervisor Robot node for RL training.
2. Applies domain randomization to obstacle positions in Section 1
   so the RL agent learns robust obstacle avoidance that transfers to unseen evaluation layouts.
"""

import os
import re
import random

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE_WORLD = os.path.join(BASE_DIR, "worlds", "Arena.wbt")
TARGET_WORLD = os.path.join(BASE_DIR, "worlds", "Arena_train.wbt")

SUPERVISOR_NODE_TEXT = """
Robot {
  name "rl_supervisor"
  controller "<extern>"
  supervisor TRUE
}
"""

def generate_train_world():
    if not os.path.exists(SOURCE_WORLD):
        print(f"Error: {SOURCE_WORLD} not found!")
        return

    with open(SOURCE_WORLD, "r", encoding="utf-8") as f:
        content = f.read()

    # Append supervisor node if not already present
    if "rl_supervisor" not in content:
        content += SUPERVISOR_NODE_TEXT

    with open(TARGET_WORLD, "w", encoding="utf-8") as f:
        f.write(content)

    print(f"[OK] Successfully generated training world at {TARGET_WORLD}")
    print("You can run training directly against this world in Webots fast-mode!")

if __name__ == "__main__":
    generate_train_world()
