# Rover and drone: reinforcement learning with ML-Agents

A truck (the rover) trained with reinforcement learning to drive to a meeting point over hilly terrain and avoid buildings and rocks. The drone is flown manually in the demonstration.



## What was trained

The rover agent (`Assets/Scripts/RoverAgent.cs`) outputs steering and throttle. It observes the direction and distance to the meeting point relative to itself, its speed and rotation, the slope it is on, and obstacles through a Ray Perception Sensor.

| Level | What it adds | Model |
|---|---|---|
| 1 | Fixed start and meeting point, flat ground | `Rover_L1.onnx` |
| 2 | Random start, heading and meeting point | `Rover_L2.onnx` |
| 3 | Procedural hills (`TerrainGenerator.cs`) | `Rover_L3.onnx` |
| 4 | Fixed buildings and rocks | `Rover_L4.onnx` |
| 5 | Random buildings and a random number of rocks | `Rover_L5.onnx` |

`Rover_L5.onnx` is the final model and also handles the earlier levels. Each level was trained starting from the previous one where the observations allowed it.

## Run the demo

1. Open the project with Unity `6000.5.9f1`. The ML-Agents package (`com.unity.ml-agents` 4.0.3) is installed from `Packages/manifest.json`.
2. Open the scene.
3. Press Play. The rover drives by itself (Behavior Type: Inference Only, model `Rover_L5`) and stays parked at the meeting point (Demo Mode).
4. Fly the drone with W, A, S, D, R and F.

## Train

Python environment (conda, Python 3.10.12):

    conda create -n mlagents python=3.10.12
    conda activate mlagents
    python -m pip install mlagents==1.1.0
    python -m pip install torch==2.2.2 onnx==1.15.0 "setuptools<81"

The torch pin matters: newer PyTorch versions break the ONNX export in `mlagents` 1.1.0.

Set Behavior Type to Default on the Truck, then from the project root:

    mlagents-learn config/rover.yaml --run-id=rover_L5 --initialize-from=rover_L4

and press Play in the Editor. Stop with Ctrl+C once, and copy `results/<run-id>/Rover.onnx` into `Assets`.

## Notes
- Braking: in agent mode, a throttle command against the direction of travel brakes the truck (`CarControl.cs`).
