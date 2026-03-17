# Distributionally Robust Safe Control of Robotic Manipulators in Dynamic Environments

> **Update (17 March 2026)**

## Description
We implement and compare two Control Barrier Function (CBF) approaches for a UR3 manipulator avoiding moving spherical obstacles, using vision-based measurements with added Gaussian noise:
- **Nominal CBF** (`src/nominal_CBF_single_trial.py`, `src/nominal_CBF_multiple_trials.py`) 
- **Distributionally Robust CBF** (`src/DR_CBF_single_trial.py`, `src/DR_CBF_multiple_trials.py`)

## Requirements
- Python: tested with Python 3.13.5
- PyBullet: tested with PyBullet 3.2.7
- NumPy: tested with Mumpy 2.1.3
- SciPy: tested with SciPy 1.15.3

## Acknowledgments
Parts of this project page were adopted from the [Nerfies](https://nerfies.github.io/) page.
