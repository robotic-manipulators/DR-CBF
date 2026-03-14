# src/config.py
import numpy as np

OBS_RADIUS = 0.08

obs_configs = [
    {"center": [0.53, 0.00, 0.26], "range": 0.01, "color": [1, 0, 0, 0.5]},
    {"center": [0.53, 0.1, 0.47],  "range": 0.01, "color": [0, 1, 0, 0.5]},
    {"center": [0.28, -0.25, 0.5], "range": 0.01, "color": [0, 0, 1, 0.5]},
    {"center": [0.0, -0.1, 0.45],  "range": 0.01, "color": [1, 1, 1, 0.5]},
    {"center": [0.22, -0.37, 0.26],"range": 0.01, "color": [0, 1, 1, 0.5]},
    {"center": [0.7, -0.2, 0.1],   "range": 0.01, "color": [0.2, 0.6, 0.5, 0.5]},
    {"center": [0.35, 0.0, 0.72],  "range": 0.01, "color": [1, 0, 1, 0.5]},
]

# control parameters
K_CBF = 10.0
GAIN = 2.5
COLLISION_THRESHOLD = 0.005
SUCCESS_THRESHOLD = 0.03
MAX_STEPS = 1000
DT = 1./240.

CAMERA_POS = [0.4, -0.9, 0.35]