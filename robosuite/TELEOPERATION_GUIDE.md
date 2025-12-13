# MultiTableAssembly Teleoperation and Data Collection Guide

This guide explains how to use keyboard teleoperation to collect demonstration data for the MultiTableAssembly environment with the Franka Panda robot.

## Overview

The teleoperation system enables you to:
- Control the Franka Panda robot using keyboard inputs
- Record RGB images from multiple cameras (agentview and robot0_eye_in_hand)
- Capture robot joint states and gripper status
- Record all custom observables (object positions, distances, etc.)
- Store demonstration data in HDF5 format for efficient storage and later playback

## Prerequisites

### Required Python Packages

```bash
pip install h5py
```

The package is required for HDF5 file format support.

## Quick Start

### 1. Test Teleoperation (No Recording)

First, verify that teleoperation works correctly:

```bash
cd /home/utk/Downloads/robosuite
python robosuite/demos/test_teleop_multitable.py
```

This will:
- Create the MultiTableAssembly environment with Panda robot
- Enable keyboard control
- Display all available observables
- Show camera images from wrist camera
- Allow you to practice control without recording data

**Keyboard Controls:**
- **Arrow Keys**: Move horizontally (Up/Down = x-axis, Left/Right = y-axis)
- **.** and **;**: Move vertically (z-axis)
- **o-p**: Rotate around z-axis (yaw)
- **y-h**: Rotate around y-axis (pitch)
- **e-r**: Rotate around x-axis (roll)
- **Spacebar**: Toggle gripper (open/close)
- **Ctrl+q**: Finish episode and exit

### 2. Collect Demonstration Data

Once you're comfortable with the controls, start collecting demonstration data:

```bash
cd /home/utk/Downloads/robosuite
python robosuite/demos/demo_collect_and_playback_data.py
```

**With custom parameters:**

```bash
python robosuite/demos/demo_collect_and_playback_data.py \
    --environment MultiTableAssembly \
    --robots Panda \
    --device keyboard \
    --directory /tmp/my_demos/ \
    --pos-sensitivity 1.0 \
    --rot-sensitivity 1.0 \
    --max_fr 20
```

**Parameters:**
- `--environment`: Environment name (default: MultiTableAssembly)
- `--robots`: Robot to use (default: Panda)
- `--device`: Input device (keyboard, spacemouse, dualsense)
- `--directory`: Where to save demonstration files (default: /tmp/teleop_demos/)
- `--pos-sensitivity`: Position control sensitivity (default: 1.0)
- `--rot-sensitivity`: Rotation control sensitivity (default: 1.0)
- `--max_fr`: Frame rate limit in fps (default: 20)

### 3. Perform the Task

The MultiTableAssembly task requires:
1. **Approach** the green cube with cone top (cubeA) on the source table
2. **Grasp** cubeA by pressing spacebar when close
3. **Lift** cubeA above the table
4. **Move** toward the red base cube (cubeB) on the destination table
5. **Align** cubeA above cubeB
6. **Lower** cubeA **onto** cubeB (make sure it's actually resting/touching)
7. **Release** by pressing spacebar to open gripper
8. **Retreat** to complete the task

**Success Criteria:**
- CubeA must be physically **resting on top** of cubeB with contact
- Horizontal alignment: centers within 4cm
- Vertical contact: cubeA bottom within 1cm of cubeB top
- Gripper must be released (not grasping)
- You'll see "✓ Task completed successfully!" when criteria are met

When you finish the episode (press Ctrl+q), the data will be automatically saved.

### 4. Playback Recorded Demonstrations

To verify your recorded demonstrations:

```bash
python robosuite/demos/demo_collect_and_playback_data.py \
    --playback /tmp/my_demos/demo_episode_0001_<timestamp>.h5
```

This will replay the recorded actions and show the robot performing the task.

**Important**: Your recordings now include comprehensive state data that will be restored during playback:
- Complete joint positions and velocities (qpos, qvel) for all objects and robots
- Body positions and rotations
- This ensures perfect reproduction of the initial configuration, with the robot arm and objects starting in exactly the same position as during recording
- Observables are automatically updated after state restoration to ensure visual rendering matches the restored state

## Data Format

Each demonstration is saved as an HDF5 file containing:

### Metadata (file attributes)
- `episode_number`: Episode count
- `environment`: Environment class name
- `robots`: List of robot names
- `total_steps`: Number of timesteps in episode
- `start_time`: Unix timestamp when episode started
- `duration`: Total duration in seconds
- `success`: Whether the episode was completed successfully

### Initial State Data (for perfect replay)

#### `/initial_state_data` (comprehensive state restoration)
- `qpos`: All joint positions (shape: N_joints), dtype=float64
- `qvel`: All joint velocities (shape: N_joints), dtype=float64
- `body_pos`: All body positions (shape: N_bodies x 3), dtype=float64
- `body_rot`: All body rotation matrices (shape: N_bodies x 3 x 3), dtype=float64

These datasets enable exact restoration of the entire simulation state during playback.

#### Fallback Data (for legacy recordings)
- `initial_state`: Flattened complete state
- `cubeA_joint_qpos`: CubeA joint positions (shape: 7)
- `cubeB_joint_qpos`: CubeB joint positions (shape: 7)
- `robot0_joint_pos`: Robot joint positions (shape: 7)

### Datasets

#### `/actions`
- Shape: `(N,)` where N is the number of steps
- Robot control actions applied at each timestep

#### `/rewards`
- Shape: `(N,)`
- Reward received at each timestep

#### `/dones`
- Shape: `(N,)`
- Boolean indicating episode completion

#### `/timestamps`
- Shape: `(N,)`
- Relative time in seconds from episode start

#### `/camera_images/<camera_name>`
- Shape: `(N, H, W, 3)` where H=256, W=256
- RGB images from each camera
- Available cameras: `agentview`, `robot0_eye_in_hand`

#### `/robot_states/<field>`
- `robot0_joint_pos`: Joint positions, shape `(N, 7)`
- `robot0_joint_vel`: Joint velocities, shape `(N, 7)`

#### `/gripper_states/<field>`
- `robot0_right_gripper_qpos`: Gripper joint positions
- `robot0_right_gripper_qvel`: Gripper joint velocities
- `robot0_right_gripper_action`: Gripper control action

#### `/observables/<field>`
All environment observables including:
- `cubeA_pos`: Position of item cube (3D)
- `cubeB_pos`: Position of base cube (3D)
- `cubeA_to_cubeB`: Distance vector between cubes
- `gripper_to_cubeA`: Gripper to cubeA distance
- `gripper_to_cubeB`: Gripper to cubeB distance
- `cubeA_to_source_table`: CubeA to source table center
- `cubeB_to_dest_table`: CubeB to destination table center
- Plus all standard robot observables (eef_pos, eef_quat, etc.)

## Loading Data in Python

Example code to load and process demonstration data:

```python
import h5py
import numpy as np

# Load demonstration
with h5py.File('demo_episode_0001_<timestamp>.h5', 'r') as f:
    # Print metadata
    print(f"Environment: {f.attrs['environment']}")
    print(f"Robot: {f.attrs['robots']}")
    print(f"Total steps: {f.attrs['total_steps']}")
    print(f"Duration: {f.attrs['duration']:.2f} seconds")
    
    # Load trajectory data
    actions = f['actions'][:]
    rewards = f['rewards'][:]
    
    # Load camera images
    agentview_images = f['camera_images/agentview'][:]
    wrist_images = f['camera_images/robot0_eye_in_hand'][:]
    
    # Load robot states
    joint_positions = f['robot_states/robot0_joint_pos'][:]
    
    # Load observables
    cubeA_positions = f['observables/cubeA_pos'][:]
    cubeB_positions = f['observables/cubeB_pos'][:]
    
    print(f"\nData shapes:")
    print(f"  Actions: {actions.shape}")
    print(f"  Agentview images: {agentview_images.shape}")
    print(f"  Wrist images: {wrist_images.shape}")
    print(f"  Joint positions: {joint_positions.shape}")
```

## Tips for Good Demonstrations

1. **Practice First**: Use the test script to get comfortable with controls before recording
2. **Smooth Motions**: Make smooth, deliberate movements rather than jerky corrections
3. **Clear Success**: Ensure the cube is properly stacked before finishing the episode
4. **Multiple Angles**: Vary your approach paths to create diverse demonstrations
5. **Frame Rate**: Keep the frame rate at 20 fps for consistent timing
6. **Check Recordings**: Use playback mode to verify demonstrations are successful

## Troubleshooting

### "ModuleNotFoundError: No module named 'h5py'"
```bash
pip install h5py
```

### Keyboard not responding
- Make sure the viewer window has focus
- On macOS, you may need to run with root access: `sudo python ...`

### Gripper not opening/closing
- Press spacebar firmly and wait a moment
- The gripper action is toggled, not held

### Robot moving too fast/slow
- Adjust `--pos-sensitivity` (default 1.0)
- Adjust `--rot-sensitivity` (default 1.0)
- Try values between 0.5 and 2.0

### Camera view not showing wrist camera
- Verify camera name: `robot0_eye_in_hand`
- Check that `use_camera_obs=True` in environment creation
- Ensure `render_camera="robot0_eye_in_hand"` is set

## Advanced Usage

### Multiple Episodes

Collect multiple demonstrations in sequence:

```bash
# Collect first demo
python robosuite/demos/demo_collect_and_playback_data.py --directory /tmp/demos_set1/

# After finishing (Ctrl+q), run again for next demo
python robosuite/demos/demo_collect_and_playback_data.py --directory /tmp/demos_set1/
```

Each episode will be saved as a separate HDF5 file with incrementing episode numbers.

### Custom Controller

Use a different controller configuration:

```bash
python robosuite/demos/demo_collect_and_playback_data.py \
    --controller OSC_POSE \
    --robots Panda
```

### Different Sensitivity Settings

For finer control:

```bash
python robosuite/demos/demo_collect_and_playback_data.py \
    --pos-sensitivity 0.5 \
    --rot-sensitivity 0.3
```

For faster control:

```bash
python robosuite/demos/demo_collect_and_playback_data.py \
    --pos-sensitivity 2.0 \
    --rot-sensitivity 1.5
```

## Next Steps

After collecting demonstrations:
1. Use the data for imitation learning
2. Train behavioral cloning policies
3. Analyze trajectory statistics
4. Create dataset visualizations
5. Benchmark policy performance

## File Locations

- **Teleoperation script**: `robosuite/demos/demo_collect_and_playback_data.py`
- **Test script**: `robosuite/demos/test_teleop_multitable.py`
- **Quick start script**: `robosuite/demos/quickstart_multi_table_panda.py`
- **Environment file**: `robosuite/environments/manipulation/multi_table_assembly.py`
- **Device control**: `robosuite/devices/keyboard.py`
- **Data directory**: `/tmp/teleop_demos/` (default)

## Contact & Support

For issues or questions:
1. Check this guide thoroughly
2. Review the test script output for diagnostic information
3. Verify all observables are accessible
4. Ensure camera images are being captured correctly
