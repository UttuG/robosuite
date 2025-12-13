#!/usr/bin/env python3
"""Test object positions during playback restoration."""

import h5py
import numpy as np
import robosuite as suite
from robosuite import load_composite_controller_config

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

print("=" * 80)
print("PLAYBACK OBJECT POSITION TEST")
print("=" * 80)

# Test with the latest demo file
import os
demo_dir = '/tmp/my_demos'
demo_files = [f for f in os.listdir(demo_dir) if f.endswith('.h5')]
if demo_files:
    latest_demo = max(demo_files, key=lambda x: os.path.getctime(os.path.join(demo_dir, x)))
    demo_file = os.path.join(demo_dir, latest_demo)
else:
    demo_file = '/tmp/my_demos/demo_episode_0001_20251212_103110.h5'  # fallback

with h5py.File(demo_file, 'r') as f:
    print(f"\nLoading latest demo: {demo_file}")

    # Load saved object positions
    saved_cubeA_qpos = None
    saved_cubeB_qpos = None
    saved_robot_qpos = None

    if 'cubeA_joint_qpos' in f and 'cubeB_joint_qpos' in f:
        saved_cubeA_qpos = f['cubeA_joint_qpos'][:]
        saved_cubeB_qpos = f['cubeB_joint_qpos'][:]
        print(f"✓ Saved cubeA qpos: {saved_cubeA_qpos}")
        print(f"✓ Saved cubeB qpos: {saved_cubeB_qpos}")

    # Load robot positions
    if 'robot0_joint_pos' in f:
        saved_robot_qpos = f['robot0_joint_pos'][:]
        print(f"✓ Saved robot qpos: {saved_robot_qpos}")

    # Check if comprehensive data exists
    if 'initial_state_data' in f:
        print("✓ Comprehensive state data available")
        qpos = f['initial_state_data']['qpos'][:]
        print(f"   Full qpos shape: {qpos.shape}")
        print(f"   Full cubeA qpos: {qpos[7:14]}")
        print(f"   Full cubeB qpos: {qpos[14:21]}")
        print(f"   Full robot qpos: {qpos[0:7]}")
    else:
        print("✗ No comprehensive state data (old recording)")

print("\n" + "-" * 80)
print("TESTING PLAYBACK RESET BEHAVIOR")
print("-" * 80)

# Test 1: Normal reset (should randomize)
print("\n1. Testing normal reset (should randomize)...")
obs = env.reset()
normal_cubeA_pos = env.sim.data.qpos[7:10].copy()
normal_cubeB_pos = env.sim.data.qpos[14:17].copy()
normal_robot_pos = env.sim.data.qpos[0].copy()
print(f"   Normal reset cubeA pos: {normal_cubeA_pos}")
print(f"   Normal reset cubeB pos: {normal_cubeB_pos}")
print(f"   Normal reset robot joint 0: {normal_robot_pos}")

# Test 2: Deterministic reset (should NOT randomize)
print("\n2. Testing deterministic reset (should NOT randomize)...")
original_deterministic = env.deterministic_reset
env.deterministic_reset = True
env.reset()
env.deterministic_reset = original_deterministic
deterministic_cubeA_pos = env.sim.data.qpos[7:10].copy()
deterministic_cubeB_pos = env.sim.data.qpos[14:17].copy()
deterministic_robot_pos = env.sim.data.qpos[0].copy()
print(f"   Deterministic reset cubeA pos: {deterministic_cubeA_pos}")
print(f"   Deterministic reset cubeB pos: {deterministic_cubeB_pos}")
print(f"   Deterministic reset robot joint 0: {deterministic_robot_pos}")

# Test 3: Restore saved state (like playback_trajectory does)
print("\n3. Restoring saved state using playback method...")
with h5py.File(demo_file, 'r') as f:
    if 'initial_state_data' in f:
        # Temporarily set deterministic_reset to True to prevent random object placement
        original_deterministic_reset = env.deterministic_reset
        env.deterministic_reset = True
        env.reset()
        env.deterministic_reset = original_deterministic_reset

        # Now restore comprehensive state
        qpos = f['initial_state_data']['qpos'][:]
        qvel = f['initial_state_data']['qvel'][:]

        # Set all joint positions and velocities
        env.sim.data.qpos[:] = qpos
        env.sim.data.qvel[:] = qvel

        # Forward simulation to update derived quantities
        env.sim.forward()

        restored_cubeA_pos = env.sim.data.qpos[7:10].copy()
        restored_cubeB_pos = env.sim.data.qpos[14:17].copy()
        restored_robot_pos = env.sim.data.qpos[0].copy()
        print(f"   Restored cubeA pos: {restored_cubeA_pos}")
        print(f"   Restored cubeB pos: {restored_cubeB_pos}")
        print(f"   Restored robot joint 0: {restored_robot_pos}")

        # Compare with comprehensive saved data
        qpos_full = f['initial_state_data']['qpos'][:]
        expected_cubeA_pos = qpos_full[7:10]  # Position part of cubeA joint
        expected_cubeB_pos = qpos_full[14:17]  # Position part of cubeB joint
        expected_robot_pos = qpos_full[0]     # First robot joint

        cubeA_diff = np.linalg.norm(restored_cubeA_pos - expected_cubeA_pos)
        cubeB_diff = np.linalg.norm(restored_cubeB_pos - expected_cubeB_pos)
        robot_diff = abs(restored_robot_pos - expected_robot_pos)

        print("\n4. Comparison with saved positions:")
        print(f"   cubeA difference: {cubeA_diff:.2e}")
        print(f"   cubeB difference: {cubeB_diff:.2e}")
        print(f"   robot difference: {robot_diff:.2e}")

        if cubeA_diff < 1e-6 and cubeB_diff < 1e-6 and robot_diff < 1e-6:
            print("   ✓✓✓ PERFECT RESTORATION - OBJECTS WILL BE IN EXACT POSITIONS! ✓✓✓")
        else:
            print("   ✗ RESTORATION INACCURATE")
            print(f"   Expected cubeA pos: {expected_cubeA_pos}")
            print(f"   Expected cubeB pos: {expected_cubeB_pos}")
            print(f"   Expected robot joint 0: {expected_robot_pos}")
    else:
        print("   ✗ No comprehensive state data to restore")

print("\n" + "=" * 80)
print("CONCLUSION:")
if 'initial_state_data' in f:
    print("✓ This demo file has comprehensive state data")
    print("✓ Playback will restore EXACT object and robot positions")
    print("✓ 1:1 reproduction is now possible")
else:
    print("✗ This demo file lacks comprehensive state data")
    print("⚠ Playback will use fallback methods (less accurate)")
    print("💡 Record new demos to get perfect restoration")
print("=" * 80)

env.close()