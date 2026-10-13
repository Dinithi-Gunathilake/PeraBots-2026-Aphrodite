# Reinforcement Learning Pipeline: PeraBots 2026

This directory contains the complete offline Reinforcement Learning (RL) training suite for the **TwoWheelRobot**.

---

## 1. Architecture Overview

* **Section 1 (Obstacle Navigation)**:
  * Handled via **Deep Reinforcement Learning (PPO)**.
  * Inputs: 8 continuous distance sensors + relative distance to goal + heading error $\sin(\Delta\theta), \cos(\Delta\theta)$.
  * Outputs: Continuous differential drive wheel speeds in range $[-10.0, 10.0]$ rad/s.
  * Inference: Pure NumPy forward pass (`rl_policy.py`), ensuring **zero external package dependencies** on the evaluation server.

* **Section 2 (Maze Exploration)**:
  * Handled via **Graph Depth-First Search (DFS)** + **Camera Tile Detection**.
  * Explores all 36 cells ($9 \times 4$ grid), records Blue/Green tiles at dead ends, identifies the majority, navigates to the Red end cell, and activates the required LED (`led_blue` or `led_green`).

---

## 2. File Organization

```
PeraBots-2026-Aphrodite/
├── controllers/
│   └── main_controller/
│       ├── controller_section1.py   # Section 1 controller using RLPolicy
│       ├── controller_section2.py   # Section 2 maze & tile detection controller
│       ├── rl_policy.py             # Pure NumPy neural network forward-pass
│       ├── policy_weights.json      # Exported weights from PPO training
│       └── main_controller.py       # (Untouched per user request)
└── training/
    ├── webots_env.py                # Webots simulation environment (Supervisor-based)
    ├── train_ppo.py                 # PyTorch PPO training script
    ├── randomize_world.py           # Domain randomization generator
    └── README.md                    # This guide
```

---

## 3. How to Train the RL Model

### Step 1: Generate the Training World
Run:
```bash
python training/randomize_world.py
```
This produces `worlds/Arena_train.wbt` containing a headless Supervisor node.

### Step 2: Launch Webots in Fast / Batch Mode
For maximum training throughput, launch Webots without graphics rendering:
```bash
webots --mode=fast --no-rendering worlds/Arena_train.wbt
```
*(Or open `worlds/Arena_train.wbt` in the Webots GUI and set the simulation speed to Fast).*

### Step 3: Run the PPO Trainer
In your terminal:
```bash
python training/train_ppo.py
```

* The trainer collects experience batches from the Webots environment.
* After every rollout, it optimizes the Actor-Critic networks.
* It automatically saves the checkpoint to `training/ppo_checkpoint.pt` and exports the lightweight NumPy weights directly to:
  `controllers/main_controller/policy_weights.json`.

---

## 4. Evaluation & Submission

When submitting:
* The robot uses `controllers/main_controller/controller_section1.py` and `controllers/main_controller/controller_section2.py`.
* `rl_policy.py` reads `policy_weights.json` with pure NumPy.
* No `supervisor TRUE` or external RL dependencies are required at evaluation time!
