# nominal_CBF_single_trial.py
import pybullet as p
import time
import numpy as np
import random
import matplotlib.pyplot as plt
import os

from src import config
from src.simulation import run_experiment

plt.rcParams['font.sans-serif'] = ['Arial']
plt.rcParams['axes.unicode_minus'] = False


def setup_realtime_plot(num_obs, obs_configs):
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlabel('Steps')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Real-time Distance to Obstacles')
    ax.grid(True)
    ax.set_ylim(-0.1, 0.6)

    lines = []
    for i in range(num_obs):
        if i == 3:
            color = (0.8, 0.8, 0.8)
        else:
            color = obs_configs[i]["color"][:3]
        line, = ax.plot([], [], label=f'Obstacle {i+1}', color=color)
        lines.append(line)

    ax.axhline(y=0, color='r', linestyle='--', linewidth=1, label='Distance=0')
    ax.legend(loc='upper right')
    fig.canvas.draw()
    plt.pause(0.5)
    return fig, ax, lines


if __name__ == "__main__":
    os.makedirs('single_trial_results', exist_ok=True)
    physicsClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(cameraDistance=1.8, cameraYaw=45,
                                 cameraPitch=-30, cameraTargetPosition=[0, 0, 0.3])

    vision_noise = 0.1
    step_size = 0.005
    print(f"Default using speed 2, step size: {step_size}")

    print("Initializing real-time plotting window...")
    fig, ax, lines = setup_realtime_plot(len(config.obs_configs), config.obs_configs)
    print("Plot window ready.")

    print("\nStarting single experiment...")
    for i in range(3, 0, -1):
        print(f"Countdown: {i} seconds")
        p.stepSimulation()
        plt.pause(1)
    print("Simulation started!")

    seed = int(time.time() * 1000) % (2**32)
    np.random.seed(seed)
    random.seed(seed)

    result, steps, dist_history = run_experiment(
        step_size=step_size,
        vision_noise=vision_noise,
        use_dr=False,
        enable_brown_spheres=False,
        plot_objects=(fig, ax, lines)
    )

    print(f"\nExperiment result: {result}")

    plt.figure(figsize=(10, 6))
    plt.rcParams.update({'font.size': 12, 'axes.labelsize': 20, 'axes.titlesize': 24,
                         'legend.fontsize': 14, 'xtick.labelsize': 18, 'ytick.labelsize': 18})

    num_obs = len(dist_history) if dist_history else 0
    for i in range(num_obs):
        if i == 3:
            color = (0.8, 0.8, 0.8)
        else:
            color = config.obs_configs[i]["color"][:3]
        plt.plot(steps, dist_history[i], label=f'Obstacle {i+1}', color=color)

    plt.axhline(y=0, color='r', linestyle='--', linewidth=1, label='Distance=0')
    plt.xlabel('Steps')
    plt.ylabel('Distance(m)')
    plt.title('Distance to Obstacles')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.ylim(bottom=-0.1, top=0.6)

    filename = os.path.join('single_trial_results', f'nominal_CBF_sigma{vision_noise}_{result}.pdf')
    plt.savefig(filename, dpi=600, bbox_inches='tight')
    plt.close()
    print(f"Final curve saved to {filename}")

    while p.isConnected():
        p.stepSimulation()
        time.sleep(0.01)