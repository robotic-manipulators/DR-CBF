# nominal_CBF_multiple_trials.py
import pybullet as p
import time
import numpy as np
import random
import os
import csv
from src import config
from src.simulation import run_experiment

if __name__ == "__main__":
    os.makedirs('multiple_trials_results', exist_ok=True)
    p.connect(p.DIRECT)

    vision_noise = 0.05
    step_size = 0.005
    print(f"Using speed 2, step size: {step_size}")

    num_trials = 100
    success_count = 0
    collision_count = 0
    timeout_count = 0

    script_name = os.path.splitext(os.path.basename(__file__))[0]
    csv_filename = os.path.join('multiple_trials_results', script_name + '.csv')
    f = open(csv_filename, 'w', newline='', encoding='utf-8')
    writer = csv.writer(f)
    writer.writerow(['Trial', 'Result'])
    f.flush()

    for trial in range(1, num_trials + 1):
        seed = int(time.time() * 1000 + trial) % (2**32)
        np.random.seed(seed)
        random.seed(seed)

        result = run_experiment(
            step_size=step_size,
            vision_noise=vision_noise,
            use_dr=False,
            enable_brown_spheres=False,
            plot_objects=None
        )

        if result == 'success':
            result_value = 1
            color = '\033[92m'
            success_count += 1
        elif result == 'collision':
            result_value = 0
            color = '\033[91m'
            collision_count += 1
        else:
            result_value = -1
            color = '\033[93m'
            timeout_count += 1

        writer.writerow([trial, result_value])
        f.flush()
        print(f"Trial {trial}/{num_trials} completed: {color}{result}\033[0m")

    f.close()

    print("\n========== Results ==========")
    print(f"Method: nominal-CBF")
    print(f"sigma={vision_noise}")
    print(f"Total trials: {num_trials}")
    print(f"Success: {success_count} ({success_count/num_trials*100:.1f}%)")
    print(f"Collision: {collision_count} ({collision_count/num_trials*100:.1f}%)")
    print(f"Timeout: {timeout_count} ({timeout_count/num_trials*100:.1f}%)")

    while p.isConnected():
        p.stepSimulation()
        time.sleep(0.01)