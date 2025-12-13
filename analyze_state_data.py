#!/usr/bin/env python3
"""Check what's in the initial_state vs comprehensive data."""

import h5py
import numpy as np

# Check the demo file
demo_file = '/tmp/my_demos/demo_episode_0001_20251212_115307.h5'

with h5py.File(demo_file, 'r') as f:
    print("=" * 80)
    print("ANALYZING STATE DATA IN DEMO FILE")
    print("=" * 80)

    if 'initial_state' in f:
        initial_state = f['initial_state'][:]
        print(f"initial_state shape: {initial_state.shape}")
        print(f"initial_state first 10 values: {initial_state[:10]}")

    if 'initial_state_data' in f:
        print("\nComprehensive state data:")
        qpos = f['initial_state_data']['qpos'][:]
        qvel = f['initial_state_data']['qvel'][:]
        body_pos = f['initial_state_data']['body_pos'][:]
        body_rot = f['initial_state_data']['body_rot'][:]

        print(f"  qpos shape: {qpos.shape} (should be 23)")
        print(f"  qvel shape: {qvel.shape} (should be 21)")
        print(f"  body_pos shape: {body_pos.shape}")
        print(f"  body_rot shape: {body_rot.shape}")

        print(f"\n  qpos values: {qpos}")
        print(f"  qvel values: {qvel[:10]}...")  # First 10

    print("\n" + "-" * 50)
    print("COMPARISON")
    print("-" * 50)

    # Check if initial_state contains the same data as our comprehensive save
    if 'initial_state' in f and 'initial_state_data' in f:
        initial_state = f['initial_state'][:]
        qpos = f['initial_state_data']['qpos'][:]
        qvel = f['initial_state_data']['qvel'][:]

        # The initial_state should contain qpos + qvel + other stuff
        expected_combined = np.concatenate([qpos, qvel])
        print(f"Expected combined qpos+qvel shape: {expected_combined.shape}")
        print(f"Actual initial_state shape: {initial_state.shape}")

        if len(expected_combined) <= len(initial_state):
            match = np.allclose(expected_combined, initial_state[:len(expected_combined)], atol=1e-10)
            print(f"qpos+qvel match with initial_state: {match}")
        else:
            print("initial_state is shorter than expected")

print("\n" + "=" * 80)
print("CONCLUSION")
print("=" * 80)
print("The initial_state field should contain the complete Mujoco state.")
print("Our comprehensive data should be equivalent.")
print("If restoration fails, we should use initial_state instead.")
print("=" * 80)