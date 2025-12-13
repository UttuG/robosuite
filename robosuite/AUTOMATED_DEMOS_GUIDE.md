# Automated Demonstration Generation Guide for MultiTableAssembly

This guide explains how to use the automated script to generate demonstration trajectories for the MultiTableAssembly environment programmatically, without manual teleoperation.

## Overview

The automated demo generation system enables you to:
- Generate demonstration data programmatically using a waypoint-based planner
- Control the Franka Panda robot using OSC_POSE controller
- Record RGB images from multiple cameras (agentview and robot0_eye_in_hand)
- Capture robot joint states, gripper status, and all observables
- Store demonstration data in HDF5 format compatible with existing playback systems
- Achieve high success rates for stacking tasks

## Prerequisites

### Required Python Packages

```bash
pip install h5py numpy
```

The packages are required for HDF5 file format support and numerical computations.

## Quick Start

### 1. Generate a Single Demo

To generate a single demonstration:

```bash
cd /home/utk/Downloads/robosuite
python robosuite/scripts/generate_automated_demos.py --num_demos 1
```

This will:
- Create the MultiTableAssembly environment with Panda robot
- Execute a waypoint-based plan: Pre-Grasp → Grasp → Lift → Place → Retreat
- Record all trajectory data, images, and states
- Save the demonstration to `/tmp/automated_demos/demo_episode_XXXX.h5`

### 2. Generate Multiple Demos

To generate multiple demonstrations (e.g., 10):

```bash
python robosuite/scripts/generate_automated_demos.py --num_demos 10
```

Each demo will use a different random initial object placement, ensuring diversity in the dataset.

## How It Works

### Waypoint-Based Planner

The script implements a 8-step sequence:

1. **Pre-Grasp**: Move above Cube A (10cm height)
2. **Grasp**: Lower to Cube A center with aligned orientation
3. **Close Gripper**: Secure the grasp
4. **Lift**: Raise to 15cm height
5. **Pre-Place**: Move above Cube B (15cm height)
6. **Place**: Lower to Cube B top (8cm height for stacking)
7. **Open Gripper**: Release the object
8. **Retreat**: Move away to safe position

### Controller Configuration

- **Controller**: OSC_POSE (Operational Space Control for position/orientation)
- **Control Frequency**: 20 Hz
- **Action Space**: 7D (3D position + 3D orientation + 1D gripper)

### Success Criteria

The script achieves success when:
- Cube A is positioned above the second table (> 0.84m height)
- Cube A is not being grasped by the gripper
- Horizontal distance between Cube A and Cube B < 4cm
- Vertical contact between Cube A bottom and Cube B top < 1cm

## Output Files

Generated HDF5 files contain:

### Metadata
- `episode_number`: Sequential episode ID
- `environment`: "MultiTableAssembly"
- `robots`: ["Panda"]
- `total_steps`: Number of timesteps in trajectory
- `start_time`/`duration`: Timing information
- `success`: Boolean success flag

### Trajectory Data
- `actions`: Control actions (7D) for each timestep
- `rewards`: Reward values
- `dones`: Episode termination flags
- `timestamps`: Relative timestamps

### State Data
- `initial_state`: Complete MuJoCo simulation state for playback
- `initial_state_data`: Joint positions, velocities, body positions/rotations
- `robot_states`: Joint positions and velocities for each timestep
- `gripper_states`: Gripper joint positions and actions
- `observables`: All environment observables

### Camera Data
- `camera_images/agentview`: RGB images from overhead camera
- `camera_images/robot0_eye_in_hand`: RGB images from wrist camera

## Playback Verification

To verify generated demos work with the playback system:

```bash
python robosuite/demos/demo_collect_and_playback_data.py --demo_file /tmp/automated_demos/demo_episode_0001_*.h5
```

This will replay the demonstration in the environment, confirming the trajectory is valid.

## Customization

### Modifying Output Directory

Edit the `output_dir` parameter in `AutomatedDemoGenerator.__init__()`:

```python
self.output_dir = "/path/to/your/demos"
```

### Adjusting Waypoints

Modify the `get_target_poses()` method to change grasp heights, approach angles, or placement positions.

### Controller Tuning

Adjust PID gains in `move_to_pose()` for different convergence behavior:

```python
kp_pos = 5.0    # Position gain
kp_ori = 2.0    # Orientation gain
```

## Troubleshooting

### Common Issues

1. **Import Errors**: Ensure all required packages are installed and the script is run from the robosuite root directory.

2. **Controller Failures**: The OSC_POSE controller requires proper configuration. If issues persist, check that `load_part_controller_config` returns valid config.

3. **Timeout Warnings**: The planner may timeout on difficult poses. Increase `max_steps` in `move_to_pose()` or adjust tolerances.

4. **Low Success Rates**: Verify that the reward function in `multi_table_assembly.py` correctly detects stacking. Check debug output for specific failure modes.

### Debug Output

The script provides detailed debug information:

```
Rewards - Reach: 0.02, Lift: 0.00, Stack: 2.00
DEBUG: Height OK: True (0.9197 > 0.8400)
DEBUG: Not Grasping: True
DEBUG: Horiz Dist: 0.0045 (< 0.04)
DEBUG: Vert Contact: True (Diff: 0.0024)
Episode Success: True
```

This helps identify why episodes may fail:
- **Height OK**: Cube A must be above second table
- **Not Grasping**: Gripper must be open
- **Horiz Dist**: Cubes must be aligned horizontally
- **Vert Contact**: Cubes must be in contact vertically

## Integration with Training

Generated HDF5 files are fully compatible with Robosuite's data collection and playback systems. They can be used for:

- Imitation learning algorithms
- Offline reinforcement learning
- Behavior cloning
- Dataset augmentation

The format matches exactly what the teleoperation system produces, ensuring seamless integration.