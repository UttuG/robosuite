"""
Demo script for multi_table_assembly environment with multiple cameras and observables.
This script demonstrates:
1. Multiple camera views including eye-in-hand cameras
2. All custom observables (gripper-to-object, object-to-object, object-to-table)
3. RGB observations from different cameras
"""

import time
import numpy as np
import robosuite as suite
from robosuite.robots import MobileRobot
from robosuite.utils.input_utils import *

MAX_FR = 25  # max frame rate for running simulation


if __name__ == "__main__":

    # Create dict to hold options that will be passed to env creation call
    options = {}

    # print welcome info
    print("Welcome to robosuite v{}!".format(suite.__version__))
    print(suite.__logo__)

    # Set environment to MultiTableAssembly
    options["env_name"] = "MultiTableAssembly"
    
    # Choose robot
    print("\nChoose a robot for the multi-table assembly task:")
    options["robots"] = choose_robots(exclude_bimanual=False, use_humanoids=True)

    # Define multiple cameras to observe from
    # Always include agentview and wrist camera
    camera_names = ["agentview", "birdview"]
    
    # Add eye-in-hand camera (wrist camera)
    # Note: Robot-specific cameras are prefixed with robot name (e.g., "robot0_eye_in_hand")
    # Panda has an eye_in_hand camera on the wrist
    if "Panda" in options["robots"][0]:
        camera_names.append("robot0_eye_in_hand")
        print("\n✓ Added Panda wrist camera: robot0_eye_in_hand")
    # GR1 has separate cameras for left and right hands
    elif "GR1" in options["robots"][0] or "Humanoid" in options["robots"][0]:
        camera_names.extend(["robot0_eye_in_right_hand", "robot0_eye_in_left_hand"])
        print("\n✓ Added GR1 hand cameras: robot0_eye_in_right_hand, robot0_eye_in_left_hand")
    
    # Initialize the task with camera observations enabled
    env = suite.make(
        **options,
        has_renderer=True,
        has_offscreen_renderer=True,
        ignore_done=True,
        use_camera_obs=True,  # Enable camera observations
        use_object_obs=True,   # Enable object state observations
        camera_names=camera_names,
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
        control_freq=20,
        renderer="mujoco",
        reward_shaping=True,
    )
    
    # Reset environment
    obs = env.reset()
    
    # Set default camera view for rendering
    env.viewer.set_camera(camera_name="agentview")
    
    # Print available observables
    print("\n" + "="*80)
    print("AVAILABLE OBSERVABLES IN ENVIRONMENT:")
    print("="*80)
    
    for obs_name, observable in env._observables.items():
        if observable.is_enabled():
            obs_value = obs.get(obs_name, None)
            if obs_value is not None:
                if isinstance(obs_value, np.ndarray):
                    print(f"  {obs_name:40s} | Shape: {str(obs_value.shape):20s} | Type: {observable.modality}")
                else:
                    print(f"  {obs_name:40s} | Value: {obs_value:20} | Type: {observable.modality}")
    
    print("\n" + "="*80)
    print("CAMERA OBSERVATIONS:")
    print("="*80)
    
    for cam_name in camera_names:
        img_key = f"{cam_name}_image"
        if img_key in obs:
            print(f"  {cam_name:30s} | Shape: {obs[img_key].shape}")
        else:
            print(f"  {cam_name:30s} | NOT AVAILABLE")
    
    print("\n" + "="*80)
    print("CUSTOM OBSERVABLES (Object-to-Object, Gripper-to-Object, Object-to-Table):")
    print("="*80)
    
    custom_obs_keys = [
        "cubeA_pos", "cubeB_pos", 
        "cubeA_to_cubeB",
        "source_table_pos", "dest_table_pos",
        "cubeA_to_source_table", "cubeA_to_dest_table", "cubeB_to_dest_table"
    ]
    
    # Add gripper-to-object observables dynamically
    for key in obs.keys():
        if "gripper_to_cube" in key:
            custom_obs_keys.append(key)
    
    for obs_key in custom_obs_keys:
        if obs_key in obs:
            obs_value = obs[obs_key]
            if isinstance(obs_value, np.ndarray):
                print(f"  {obs_key:40s} | Value: {obs_value}")
    
    print("\n" + "="*80)
    print("Starting simulation... Press Ctrl+C to stop")
    print("="*80)
    
    # Disable certain robot parts if mobile robot
    for robot in env.robots:
        if isinstance(robot, MobileRobot):
            robot.enable_parts(legs=False, base=False)
    
    # Run simulation with random actions
    step_count = 0
    try:
        for i in range(10000):
            start = time.time()
            
            # Take random action
            action = np.random.randn(*env.action_spec[0].shape) * 0.1  # Scaled down for smoother motion
            obs, reward, done, info = env.step(action)
            env.render()
            
            # Print information every 100 steps
            if step_count % 100 == 0:
                print(f"\nStep {step_count}:")
                print(f"  Reward: {reward:.4f}")
                
                # Print key observables
                if "cubeA_pos" in obs and "cubeB_pos" in obs:
                    cubeA_pos = obs["cubeA_pos"]
                    cubeB_pos = obs["cubeB_pos"]
                    distance = np.linalg.norm(cubeA_pos - cubeB_pos)
                    print(f"  CubeA position: {cubeA_pos}")
                    print(f"  CubeB position: {cubeB_pos}")
                    print(f"  Distance A->B: {distance:.4f}")
                
                # Check if any gripper-to-cube observations exist
                for key in obs.keys():
                    if "gripper_to_cubeA" in key:
                        gripper_dist = np.linalg.norm(obs[key])
                        print(f"  {key}: {gripper_dist:.4f}")
            
            step_count += 1
            
            # Reset if done
            if done:
                print(f"\nEpisode finished at step {step_count}")
                obs = env.reset()
                step_count = 0
            
            # Limit frame rate if necessary
            elapsed = time.time() - start
            diff = 1 / MAX_FR - elapsed
            if diff > 0:
                time.sleep(diff)
                
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
    
    print("\nClosing environment...")
    env.close()
