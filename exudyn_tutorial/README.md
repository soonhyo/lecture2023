# exudyn_tutorial

## Installation
It is assumed that you have already installed Python Version 3.8 or higher, as well as the pip manager.

First we need to install the necessary packages:
- `exudyn`: base package
- `matplotlib`: for plotting sensor data
- `ngsolve`: for FEM computation
- `scipy`: needed for `ngsolve` package
```
pip install exudyn matplotlib ngsolve scipy
```

## Run Examples
Use python to run the provided examples, e.g.
```
python double_pendulum.py
```
A separate window should open with the assembled multibody system. Press `SPACE` to start the simulation.
