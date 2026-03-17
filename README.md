# Distributionally Robust Safe Control of Robotic Manipulators in Dynamic Environments

> **Update (17 March 2026)**

## Description
We implement and compare two Control Barrier Function (CBF) approaches for a UR3 manipulator avoiding moving spherical obstacles, using vision-based measurements with added Gaussian noise:
- **Nominal CBF** (`src/nominal_CBF_single_trial.py`, `src/nominal_CBF_multiple_trials.py`) 
- **Distributionally Robust CBF** (`src/DR_CBF_single_trial.py`, `src/DR_CBF_multiple_trials.py`)

## Requirements
- Python >=3.9–3.13 (tested with Python 3.13.5)
- PyBullet >=3.2.7
- NumPy >=2.0
- SciPy >=1.7

## Acknowledgments
Parts of this project page were adopted from the [Nerfies](https://nerfies.github.io/) page.
