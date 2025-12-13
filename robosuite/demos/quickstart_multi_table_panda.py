"""
Quick start script for MultiTableAssembly with Franka Panda robot.
Demonstrates eye-in-hand camera and all observables.
"""

import numpy as np
import robosuite as suite


def main():
    print("="*80)
    print("MultiTableAssembly Environment - Franka Panda Quick Start")
    print("="*80)
    
    # Create environment with Panda robot and wrist camera
    env = suite.make(
        "MultiTableAssembly",
        robots="Panda",                    # Franka Panda robot
        has_renderer=True,                 # Show visualization
        has_offscreen_renderer=True,
        use_camera_obs=True,               # Enable camera observations
        use_object_obs=True,               # Enable object state observations
        render_camera="robot0_eye_in_hand", # Default view: wrist camera
        camera_names=[
            "agentview",                   # Main view
            "robot0_eye_in_hand",          # Wrist camera (eye-in-hand)
        ],
        camera_heights=256,
        camera_widths=256,
        control_freq=20,
        reward_shaping=True,               # Dense reward for learning
        horizon=500,
    )
    
    # Reset environment
    obs = env.reset()
    
    print("\n" + "="*80)
    print("ENVIRONMENT INITIALIZED")
    print("="*80)
    
    # Show available observables
    print("\nKey Observables:")
    print("-"*80)
    
    key_obs = [
        "cubeA_pos", "cubeB_pos", "cubeA_to_cubeB",
        "gripper_to_cubeA", "gripper_to_cubeB",
        "cubeA_to_source_table", "cubeB_to_dest_table"
    ]
    
    for obs_key in key_obs:
        if obs_key in obs:
            print(f"  {obs_key:30s}: {obs[obs_key]}")
    
    # Show cameras
    print("\nAvailable Camera Views:")
    print("-"*80)
    for key in obs.keys():
        if "image" in key:
            print(f"  {key:30s}: Shape {obs[key].shape}")
    
    print("\n" + "="*80)
    print("Running simulation... (Press Ctrl+C to stop)")
    print("="*80)
    
    # Run simulation
    for i in range(1000):
        # Random action (you can replace with your policy)
        action = np.random.randn(env.action_dim) * 0.05
        
        obs, reward, done, info = env.step(action)
        env.render()
        
        # Print info every 50 steps
        if i % 50 == 0:
            cubeA_pos = obs.get("cubeA_pos", None)
            cubeB_pos = obs.get("cubeB_pos", None)
            
            if cubeA_pos is not None and cubeB_pos is not None:
                distance = np.linalg.norm(cubeA_pos - cubeB_pos)
                print(f"Step {i:4d} | Reward: {reward:6.3f} | Distance A->B: {distance:.4f}m")
        
        if done:
            print(f"\nTask completed at step {i}!")
            obs = env.reset()
    
    env.close()
    print("\nSimulation complete!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user.")
