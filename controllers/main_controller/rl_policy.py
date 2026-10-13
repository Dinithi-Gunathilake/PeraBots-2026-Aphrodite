"""
rl_policy.py

Lightweight, zero-dependency policy inference for TwoWheelRobot.
Computes the forward pass using pure NumPy so it can execute in any
Python environment without requiring PyTorch or Gymnasium at evaluation time.
"""

import math
import json
import os
import numpy as np

class RLPolicy:
    def __init__(self, weights_path=None):
        self.weights_path = weights_path or os.path.join(os.path.dirname(__file__), "policy_weights.json")
        self.loaded = False
        self.w1 = None
        self.b1 = None
        self.w2 = None
        self.b2 = None
        self.w_out = None
        self.b_out = None
        self.load_weights()

    def load_weights(self):
        """Loads weights from a JSON file if available."""
        if os.path.exists(self.weights_path):
            try:
                with open(self.weights_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                self.w1 = np.array(data["w1"], dtype=np.float32)
                self.b1 = np.array(data["b1"], dtype=np.float32)
                self.w2 = np.array(data["w2"], dtype=np.float32)
                self.b2 = np.array(data["b2"], dtype=np.float32)
                self.w_out = np.array(data["w_out"], dtype=np.float32)
                self.b_out = np.array(data["b_out"], dtype=np.float32)
                self.loaded = True
                print(f"[RLPolicy] Successfully loaded policy weights from {self.weights_path}")
            except Exception as e:
                print(f"[RLPolicy WARNING] Failed to load {self.weights_path}: {e}")
                self.loaded = False
        else:
            self.loaded = False

    def predict(self, observation):
        """
        Computes forward pass of the MLP policy:
        input (11,) -> Dense(64, tanh) -> Dense(64, tanh) -> Output(2, tanh)
        Returns action in [-1.0, 1.0]^2 representing [v_left_norm, v_right_norm].
        """
        if not self.loaded:
            return self._heuristic_fallback(observation)

        obs = np.array(observation, dtype=np.float32)
        h1 = np.tanh(np.dot(obs, self.w1) + self.b1)
        h2 = np.tanh(np.dot(h1, self.w2) + self.b2)
        action = np.tanh(np.dot(h2, self.w_out) + self.b_out)
        return float(action[0]), float(action[1])

    def _heuristic_fallback(self, obs):
        """
        Braitenberg-style fallback if trained weights are not yet present.
        obs: [d0, d1, d2, d3, d4, d5, d6, d7, norm_dist, sin_e, cos_e]
        """
        # obs[0]: front, obs[1]: front-left, obs[7]: front-right
        d_f = obs[0]
        d_fl = obs[1]
        d_fr = obs[7]
        sin_e = obs[9]

        base_speed = 0.7
        turn = sin_e * 0.6

        # Repulsive steering away from close obstacles
        if d_fl < 0.25:
            turn -= (0.25 - d_fl) * 2.5
        if d_fr < 0.25:
            turn += (0.25 - d_fr) * 2.5
        if d_f < 0.20:
            base_speed *= 0.4
            turn += 0.5 if d_fl > d_fr else -0.5

        v_l = max(-1.0, min(1.0, base_speed - turn))
        v_r = max(-1.0, min(1.0, base_speed + turn))
        return v_l, v_r
