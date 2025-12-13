# Using Franka Panda with MultiTableAssembly

## Quick Start

The Franka Panda is the recommended robot for the MultiTableAssembly environment.

### Minimal Example
```python
import robosuite as suite

env = suite.make(
    "MultiTableAssembly",
    robots="Panda",
    use_camera_obs=True,
    use_object_obs=True,
)

obs = env.reset()
```

## Panda's Eye-in-Hand Camera

The Panda robot has a **wrist-mounted camera** that provides an eye-in-hand view of the manipulation task.

### Camera Location
- **Name**: `eye_in_hand`
- **Access**: `robot0_eye_in_hand` (with robot prefix)
- **Position**: Mounted on the wrist (link7/right_hand), positioned at the end-effector
- **Orientation**: Points forward from the gripper
- **FOV**: 75 degrees

### Using the Wrist Camera
```python
env = suite.make(
    "MultiTableAssembly",
    robots="Panda",
    use_camera_obs=True,
    camera_names=["agentview", "robot0_eye_in_hand"],
    camera_heights=256,
    camera_widths=256,
)

obs = env.reset()

# Access the wrist camera image
wrist_image = obs["robot0_eye_in_hand_image"]  # Shape: (256, 256, 3)
```

## All Available Observables

### Object States
```python
cubeA_pos = obs["cubeA_pos"]          # Green cone-cube (to pick)
cubeA_quat = obs["cubeA_quat"]        # Orientation
cubeB_pos = obs["cubeB_pos"]          # Red cube base (target)
cubeB_quat = obs["cubeB_quat"]        # Orientation
```

### Relative Positions
```python
# Object to object
cube_distance = obs["cubeA_to_cubeB"]  # Vector from A to B

# Gripper to objects
gripper_to_A = obs["gripper_to_cubeA"]
gripper_to_B = obs["gripper_to_cubeB"]

# Objects to tables
A_to_source = obs["cubeA_to_source_table"]
A_to_dest = obs["cubeA_to_dest_table"]
B_to_dest = obs["cubeB_to_dest_table"]
```

### Table Positions
```python
source_table = obs["source_table_pos"]  # Left table
dest_table = obs["dest_table_pos"]      # Right table
```

### Camera Observations
```python
agentview = obs["agentview_image"]              # Main camera
birdview = obs["birdview_image"]                # Top-down
eye_in_hand = obs["robot0_eye_in_hand_image"]   # Wrist camera
robotview = obs["robot0_robotview_image"]       # Robot perspective
```

## Task Description

**Objective**: Pick the green cone-cube (cubeA) from the source table and stack it on the red cube (cubeB) on the destination table.

### Task Setup
- **Source Table** (left): Contains cubeA (green cone-cube object)
- **Destination Table** (right): Contains cubeB (red cube base)
- **Robot**: Positioned between the two tables

### Reward Structure

#### Dense Rewards (reward_shaping=True)
```python
env = suite.make(..., reward_shaping=True)
```
- **Reaching** [0, 0.25]: Encourages moving gripper toward cubeA
- **Grasping** {0, 0.25}: Bonus for grasping cubeA
- **Lifting** {0, 1.0}: Reward for lifting cubeA above source table
- **Aligning** [0, 0.5]: Encourages aligning cubeA over cubeB
- **Stacking** {0, 2.0}: Final reward for successfully stacking

#### Sparse Rewards (reward_shaping=False)
```python
env = suite.make(..., reward_shaping=False)
```
- **Success**: 2.0 when cubeA is stacked on cubeB
- **Failure**: 0.0 otherwise

## Complete Example

```python
import numpy as np
import robosuite as suite

# Create environment
env = suite.make(
    "MultiTableAssembly",
    robots="Panda",
    has_renderer=True,
    use_camera_obs=True,
    use_object_obs=True,
    camera_names=["agentview", "robot0_eye_in_hand"],
    camera_heights=256,
    camera_widths=256,
    reward_shaping=True,
    control_freq=20,
    horizon=500,
)

# Reset and run
obs = env.reset()

for step in range(500):
    # Your policy here
    action = np.random.randn(env.action_dim) * 0.05
    
    obs, reward, done, info = env.step(action)
    env.render()
    
    # Access observables
    cube_distance = np.linalg.norm(obs["cubeA_to_cubeB"])
    gripper_dist = np.linalg.norm(obs["gripper_to_cubeA"])
    
    print(f"Step {step}: Reward={reward:.3f}, Cube dist={cube_distance:.4f}")
    
    if done:
        print("Task completed!")
        break

env.close()
```

## Running Demo Scripts

### Quick Start Demo
```bash
cd /home/utk/Downloads/robosuite
python robosuite/demos/quickstart_multi_table_panda.py
```

### Full Camera & Observable Test
```bash
python robosuite/demos/test_multi_table_assembly_observables.py
```

### Interactive Camera Demo
```bash
python robosuite/demos/demo_multi_table_assembly_cameras.py
```

## Troubleshooting

### Camera not showing
- Ensure `use_camera_obs=True`
- Camera name must include robot prefix: `robot0_eye_in_hand`

### Observable returning zeros
- Ensure `use_object_obs=True`
- Check that observable name is correct

### Low frame rate with cameras
- Reduce camera resolution: `camera_heights=128, camera_widths=128`
- Use fewer cameras
- Set `has_renderer=False` for faster training

## Tips for Learning

1. **Start with dense rewards**: Use `reward_shaping=True` for easier learning
2. **Use wrist camera**: The `robot0_eye_in_hand` provides valuable visual feedback
3. **Monitor distances**: Track `gripper_to_cubeA` and `cubeA_to_cubeB` for debugging
4. **Adjust control frequency**: Higher `control_freq` gives finer control but slower simulation
5. **Use object observations**: Combine visual and state observations for best results
