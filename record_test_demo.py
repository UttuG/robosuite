#!/usr/bin/env python3
"""Quick demo recording for testing comprehensive state saving."""

import os
import time
import numpy as np
import robosuite as suite
from robosuite import load_composite_controller_config
from robosuite.demos.demo_collect_and_playback_data import HDF5DataCollectionWrapper

# Create environment
controller_config = load_composite_controller_config(
    controller=None,
    robot="Panda",
)

env = suite.make(
    "MultiTableAssembly",
    robots="Panda",
    controller_configs=controller_config,
    has_renderer=False,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    use_object_obs=True,
    ignore_done=False,
    control_freq=20,
)

# Create data collector
data_directory = "/tmp/my_demos"
os.makedirs(data_directory, exist_ok=True)
data_collector = HDF5DataCollectionWrapper(env, data_directory)

print("=" * 80)
print("RECORDING NEW DEMO WITH COMPREHENSIVE STATE SAVING")
print("=" * 80)

# Start episode
data_collector.start_episode()

# Reset environment to get initial state
obs = env.reset()

# Record a few steps with random actions
print("Recording trajectory...")
for step in range(10):
    # Random action
    action = np.random.uniform(-0.1, 0.1, size=env.action_dim)

    # Step environment
    obs, reward, done, info = env.step(action)

    # Record step
    data_collector.record_step(obs, action, reward, done, info)

    print(f"Step {step}: reward={reward:.3f}, done={done}")

# Save episode
data_collector.save_episode()

print("\n" + "=" * 80)
print("DEMO RECORDED SUCCESSFULLY")
print("Now testing playback with the new comprehensive state saving...")
print("=" * 80)

# Find the latest demo file
demo_files = [f for f in os.listdir(data_directory) if f.endswith('.h5')]
if demo_files:
    latest_demo = max(demo_files, key=lambda x: os.path.getctime(os.path.join(data_directory, x)))
    demo_path = os.path.join(data_directory, latest_demo)
    print(f"Latest demo: {demo_path}")

    # Test playback
    import h5py
    with h5py.File(demo_path, 'r') as f:
        if 'initial_state_data' in f:
            print("✓ COMPREHENSIVE STATE DATA FOUND!")
            qpos = f['initial_state_data']['qpos'][:]
            print(f"  Full qpos shape: {qpos.shape}")
            print(f"  CubeA qpos: {qpos[7:14]}")
            print(f"  CubeB qpos: {qpos[14:21]}")
            print(f"  Robot qpos: {qpos[0:7]}")
        else:
            print("✗ No comprehensive state data")

env.close()
print("\nDemo recording complete!")