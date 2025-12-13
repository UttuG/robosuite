# MultiTableAssembly Environment - Observables and Camera Setup

## Recommended Robot: Franka Panda

The **Franka Panda** robot is the primary robot for the MultiTableAssembly environment. It comes with:
- A wrist-mounted camera (`robot0_eye_in_hand`) for eye-in-hand observations
- Precise 7-DOF manipulation capabilities
- Excellent gripper for pick-and-place tasks

## Summary of Changes

### 1. **Enhanced Observables in multi_table_assembly.py**

Added comprehensive observables for better environment observation:

#### Object-to-Object Observables
- `cubeA_to_cubeB`: 3D vector from cubeA to cubeB position

#### Object-to-Table Observables
- `source_table_pos`: Position of the source table (where cubeA starts)
- `dest_table_pos`: Position of the destination table (where cubeB is located)
- `cubeA_to_source_table`: 3D vector from cubeA to source table
- `cubeA_to_dest_table`: 3D vector from cubeA to destination table  
- `cubeB_to_dest_table`: 3D vector from cubeB to destination table

#### Gripper-to-Object Observables (automatically generated per arm)
- `gripper_to_cubeA`: 3D vector from gripper to cubeA
- `gripper_to_cubeB`: 3D vector from gripper to cubeB

All observables are properly registered with:
- Correct modality (`"object"`)
- Proper sampling rate (`self.control_freq`)
- Observable class instantiation for automatic caching and updates

### 2. **Eye-in-Hand Camera Support**

The GR1 robot already has eye-in-hand cameras defined in its robot.xml:
- `eye_in_right_hand`: Located on the right hand/wrist
- `eye_in_left_hand`: Located on the left hand/wrist

**Important Note**: When using these cameras, they must be prefixed with the robot name:
- Correct: `"robot0_eye_in_right_hand"`, `"robot0_eye_in_left_hand"`
- Incorrect: `"eye_in_right_hand"`, `"eye_in_left_hand"`

### 3. **Test Scripts Created**

#### `test_multi_table_assembly_observables.py`
A comprehensive testing script that verifies:
- All camera observations are available and have correct shapes
- All object observables are present and properly computed
- Gripper-to-object observables are dynamically generated
- Observable values update correctly during simulation
- GR1 eye-in-hand cameras work with proper naming

#### `demo_multi_table_assembly_cameras.py`
An interactive demo script that:
- Shows all available observables with their values
- Displays camera observations from multiple viewpoints
- Prints real-time information about object positions and distances
- Supports multiple robots including GR1 with eye-in-hand cameras

## How to Use Camera Observations

### Basic Setup (Standard Cameras with Panda)
```python
env = suite.make(
    "MultiTableAssembly",
    robots="Panda",
    use_camera_obs=True,
    camera_names=["agentview", "birdview", "frontview"],
    camera_heights=256,
    camera_widths=256,
)
```

### With Eye-in-Hand Camera (Panda Robot - Recommended)
```python
env = suite.make(
    "MultiTableAssembly",
    robots="Panda",
    use_camera_obs=True,
    camera_names=[
        "agentview",
        "birdview",
        "robot0_eye_in_hand",  # Wrist-mounted camera
    ],
    camera_heights=256,
    camera_widths=256,
)
```

### Alternative: With Eye-in-Hand Cameras (GR1 Robot)
```python
env = suite.make(
    "MultiTableAssembly",
    robots="GR1",
    use_camera_obs=True,
    camera_names=[
        "agentview",
        "robot0_eye_in_right_hand",  # Right hand camera
        "robot0_eye_in_left_hand"     # Left hand camera
    ],
    camera_heights=256,
    camera_widths=256,
)
```

### Accessing Camera Observations
```python
obs = env.reset()

# RGB images are accessed with "_image" suffix
agentview_rgb = obs["agentview_image"]           # Shape: (256, 256, 3)
eye_in_hand_rgb = obs["robot0_eye_in_hand_image"]  # Shape: (256, 256, 3) - Panda wrist camera
```

## How to Access Custom Observables

### Object Positions and Orientations
```python
cubeA_pos = obs["cubeA_pos"]     # 3D position
cubeA_quat = obs["cubeA_quat"]   # Quaternion orientation (xyzw)
cubeB_pos = obs["cubeB_pos"]     # 3D position
cubeB_quat = obs["cubeB_quat"]   # Quaternion orientation (xyzw)
```

### Relative Distances
```python
# Object-to-object
cube_distance_vector = obs["cubeA_to_cubeB"]  # 3D vector from A to B

# Object-to-table
cubeA_to_source = obs["cubeA_to_source_table"]  # Vector to source table
cubeA_to_dest = obs["cubeA_to_dest_table"]      # Vector to destination table

# Gripper-to-object (varies by robot)
gripper_to_cubeA = obs["gripper_to_cubeA"]      # For single-arm robots
# or
gripper_to_cubeA = obs["right_gripper_to_cubeA"] # For bimanual robots
```

## Observable Modalities

The environment uses two main modality types:

1. **`"object"`** modality: Object states, positions, and relative distances
2. **`"image"`** modality: RGB camera observations (and depth if enabled)

Each modality can be enabled/disabled independently:
```python
env = suite.make(
    "MultiTableAssembly",
    use_object_obs=True,   # Enable object observables
    use_camera_obs=True,   # Enable camera observables
)
```

## Available Cameras by Robot

### Standard Cameras (All Robots)
- `agentview`: Front view of the scene
- `birdview`: Top-down view
- `frontview`: Front-facing view
- `sideview`: Side view

### Robot-Specific Cameras (Panda - Recommended Robot)
- `robot0_robotview`: Camera positioned for robot's perspective
- `robot0_eye_in_hand`: **Wrist-mounted camera** (eye-in-hand view)

### Robot-Specific Cameras (GR1)
- `robot0_robotview`: Camera on robot's head
- `robot0_obs_hands`: Observes both hands
- `robot0_overshoulder`: Over-shoulder view
- `robot0_behindhead`: Behind head view
- `robot0_eye_in_right_hand`: Right hand/wrist camera
- `robot0_eye_in_left_hand`: Left hand/wrist camera

## Running the Test Scripts

### Quick Test (Panda Robot)
```bash
cd /home/utk/Downloads/robosuite
python robosuite/demos/test_multi_table_assembly_observables.py
```

### Interactive Demo (Choose Robot)
```bash
cd /home/utk/Downloads/robosuite
python robosuite/demos/demo_multi_table_assembly_cameras.py
```

## Key Implementation Details

1. **Observable Registration**: All observables are registered in `_setup_observables()` method with proper sampling rates
2. **Sensor Decorator**: Each observable uses the `@sensor(modality=...)` decorator
3. **Observation Cache**: Observables can reference each other through `obs_cache` for efficient computation
4. **Camera Naming**: Robot-specific cameras use `{robot_name}_{camera_name}` format
5. **Dynamic Generation**: Gripper-to-object observables are generated dynamically based on robot arms

## Troubleshooting

**Issue**: Camera not found error
- **Solution**: Check if camera name has robot prefix (e.g., `robot0_` for first robot)

**Issue**: Observable returns zeros
- **Solution**: Verify `use_object_obs=True` is set when creating environment

**Issue**: GR1 cameras not available
- **Solution**: Ensure GR1 robot model is properly installed in your robosuite installation
