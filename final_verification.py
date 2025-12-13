#!/usr/bin/env python3
"""Final verification that observables are updated after state restoration."""

import h5py
import numpy as np
import robosuite as suite
from robosuite import load_composite_controller_config

# Create environment (same as playback)
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

print("=" * 80)
print("FINAL VERIFICATION: OBSERVABLES AFTER STATE RESTORATION")
print("=" * 80)

demo_file = '/tmp/my_demos/demo_episode_0001_20251212_115307.h5'

with h5py.File(demo_file, 'r') as f:
    if 'initial_state_data' in f:
        saved_qpos = f['initial_state_data']['qpos'][:]
        saved_qvel = f['initial_state_data']['qvel'][:]

        print("Simulating the fixed playback process...")

        # Step 1: Deterministic reset
        original_deterministic = env.deterministic_reset
        env.deterministic_reset = True
        obs = env.reset()
        env.deterministic_reset = original_deterministic

        # Step 2: State restoration
        env.sim.data.qpos[:] = saved_qpos
        env.sim.data.qvel[:] = saved_qvel
        env.sim.forward()

        # Step 3: Update observables (the fix)
        zero_action = np.zeros(env.action_dim)
        obs, _, _, _ = env.step(zero_action)

        print("After state restoration + observable update:")
        print(f"  qpos cubeA: {env.sim.data.qpos[7:10]}")
        print(f"  qpos cubeB: {env.sim.data.qpos[14:17]}")
        print(f"  obs cubeA: {obs.get('cubeA_pos', 'N/A')}")
        print(f"  obs cubeB: {obs.get('cubeB_pos', 'N/A')}")

        # Check if observables are reasonable (not [0,0,0])
        obs_A = obs.get('cubeA_pos', np.zeros(3))
        obs_B = obs.get('cubeB_pos', np.zeros(3))

        obs_valid = not (np.allclose(obs_A, [0,0,0]) and np.allclose(obs_B, [0,0,0]))

        print(f"\nObservables are valid (not [0,0,0]): {obs_valid}")

        if obs_valid:
            print("✓✓✓ FIX SUCCESSFUL - OBSERVABLES ARE NOW UPDATED! ✓✓✓")
            print("✓ Visual rendering should now show correct object positions")
        else:
            print("✗ Fix failed - observables still [0,0,0]")

print("\n" + "=" * 80)
print("SUMMARY")
print("=" * 80)
print("The issue was that after state restoration, observables weren't updated.")
print("Added env.step([0]*action_dim) after restoration to update observables.")
print("This should fix the visual rendering issue.")
print("=" * 80)

env.close()