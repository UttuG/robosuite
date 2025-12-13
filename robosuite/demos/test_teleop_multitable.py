"""
Simple test script for MultiTableAssembly teleoperation data collection.
Tests without HDF5 to verify basic functionality first.
"""

import numpy as np
import robosuite as suite
from robosuite import load_composite_controller_config
from robosuite.wrappers import VisualizationWrapper

# Create environment
env = suite.make(
    "MultiTableAssembly",
    robots="Panda",
    has_renderer=True,
    has_offscreen_renderer=True,
    use_camera_obs=True,
    use_object_obs=True,
    camera_names=[
        "agentview",
        "robot0_eye_in_hand",
    ],
    camera_heights=256,
    camera_widths=256,
    ignore_done=False,
    reward_shaping=True,
    control_freq=20,
    render_camera="robot0_eye_in_hand",
)

# Wrap with visualization
env = VisualizationWrapper(env, indicator_configs=None)

# Initialize keyboard device
from robosuite.devices import Keyboard

device = Keyboard(env=env, pos_sensitivity=1.0, rot_sensitivity=1.0)
env.viewer.add_keypress_callback(device.on_press)

print("="*80)
print("TEST: MultiTableAssembly Teleoperation")
print("="*80)
print("Environment created successfully")
print(f"Robot: {env.unwrapped.robots[0].name}")
print(f"Cameras: {env.unwrapped.camera_names}")
print()

# Reset environment
obs = env.reset()
env.render()

print("Testing data access:")
print("-"*80)

# Test robot state access
robot = env.unwrapped.robots[0]
print(f"Robot joint positions shape: {robot._joint_positions.shape}")
print(f"Robot joint velocities shape: {robot._joint_velocities.shape}")

# Test gripper state access
for arm in robot.arms:
    if robot.has_gripper[arm]:
        gripper_qpos = np.array([env.sim.data.qpos[x] for x in robot._ref_gripper_joint_pos_indexes[arm]])
        gripper_qvel = np.array([env.sim.data.qvel[x] for x in robot._ref_gripper_joint_vel_indexes[arm]])
        print(f"Gripper {arm} qpos: {gripper_qpos}")
        print(f"Gripper {arm} qvel: {gripper_qvel}")
        print(f"Gripper {arm} current_action: {robot.gripper[arm].current_action}")

# Test camera observations
print(f"\nCamera observations:")
for cam_name in env.unwrapped.camera_names:
    image_key = f"{cam_name}_image"
    if image_key in obs:
        print(f"  {image_key}: shape {obs[image_key].shape}")

# Test custom observables
print(f"\nCustom observables:")
custom_obs = [
    'cubeA_pos', 'cubeB_pos', 'cubeA_to_cubeB',
    'gripper_to_cubeA', 'gripper_to_cubeB',
    'cubeA_to_source_table', 'cubeB_to_dest_table'
]
for obs_key in custom_obs:
    if obs_key in obs:
        print(f"  {obs_key}: {obs[obs_key]}")

print("\n" + "="*80)
print("Starting teleoperation... Press Ctrl+q to finish")
print("="*80)

# Initialize device control
device.start_control()
all_prev_gripper_actions = [
    {
        f"{robot_arm}_gripper": np.repeat([0], robot.gripper[robot_arm].dof)
        for robot_arm in robot.arms
        if robot.gripper[robot_arm].dof > 0
    }
    for robot in env.unwrapped.robots
]

step_count = 0

# Simple control loop
try:
    while True:
        # Set active robot
        active_robot = env.unwrapped.robots[device.active_robot]

        # Get the newest action
        input_ac_dict = device.input2action()

        # If action is none, then this a reset so we should break
        if input_ac_dict is None:
            print(f"\nReset requested. Total steps: {step_count}")
            break

        from copy import deepcopy
        from robosuite.controllers.composite.composite_controller import WholeBody

        action_dict = deepcopy(input_ac_dict)
        
        # Set arm actions
        for arm in active_robot.arms:
            if isinstance(active_robot.composite_controller, WholeBody):
                controller_input_type = active_robot.composite_controller.joint_action_policy.input_type
            else:
                controller_input_type = active_robot.part_controllers[arm].input_type

            if controller_input_type == "delta":
                action_dict[arm] = input_ac_dict[f"{arm}_delta"]
            elif controller_input_type == "absolute":
                action_dict[arm] = input_ac_dict[f"{arm}_abs"]
            else:
                raise ValueError

        # Maintain gripper state
        env_action = [robot.create_action_vector(all_prev_gripper_actions[i]) for i, robot in enumerate(env.unwrapped.robots)]
        env_action[device.active_robot] = active_robot.create_action_vector(action_dict)
        env_action = np.concatenate(env_action)
        
        for gripper_ac in all_prev_gripper_actions[device.active_robot]:
            all_prev_gripper_actions[device.active_robot][gripper_ac] = action_dict[gripper_ac]

        # Step environment
        obs, reward, done, info = env.step(env_action)
        env.render()
        step_count += 1

        # Check for task success
        success = env.unwrapped._check_success()
        
        # Print status every 50 steps
        if step_count % 50 == 0:
            print(f"Step {step_count} | Reward: {reward:.3f} | Done: {done} | Success: {success}")

        if done:
            print(f"\nHorizon reached at step {step_count}!")
            if success:
                print("✓ Task completed successfully!")
            break
        
        if success:
            print(f"\n✓ Task completed successfully at step {step_count}!")
            print("Continue playing or press Ctrl+q to finish...")

except KeyboardInterrupt:
    print(f"\n\nInterrupted by user at step {step_count}")

env.close()
print("\nTest complete!")
