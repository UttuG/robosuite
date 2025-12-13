#!/usr/bin/env python3
"""Debug playback state restoration."""

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
    has_renderer=False,  # No rendering for debugging
    has_offscreen_renderer=False,
    use_camera_obs=False,
    use_object_obs=True,
    ignore_done=False,
    control_freq=20,
)

print("=" * 80)
print("DEBUGGING PLAYBACK STATE RESTORATION")
print("=" * 80)

# Use the file the user mentioned
demo_file = '/tmp/my_demos/demo_episode_0001_20251212_115307.h5'

with h5py.File(demo_file, 'r') as f:
    print(f"\nAnalyzing demo file: {demo_file}")

    # Check what data is available
    print("\nAvailable data keys:")
    def print_keys(name, obj):
        if isinstance(obj, h5py.Dataset):
            print(f"  {name}: {obj.shape} {obj.dtype}")
        else:
            print(f"  {name}/")
    f.visititems(print_keys)

    # Check comprehensive state data
    if 'initial_state_data' in f:
        print("\n✓ Comprehensive state data found")
        initial_state_data = f['initial_state_data']
        qpos = initial_state_data['qpos'][:]
        qvel = initial_state_data['qvel'][:]

        print(f"  Saved qpos shape: {qpos.shape}")
        print(f"  Saved robot joints (0-6): {qpos[0:7]}")
        print(f"  Saved cubeA joints (7-13): {qpos[7:14]}")
        print(f"  Saved cubeB joints (14-20): {qpos[14:21]}")

        # Simulate the exact playback restoration process
        print("\n" + "-" * 50)
        print("SIMULATING PLAYBACK RESTORATION PROCESS")
        print("-" * 50)

        # Step 1: Normal reset (what happens without deterministic_reset)
        print("\n1. Normal reset (randomizes objects):")
        obs = env.reset()
        normal_cubeA_pos = env.sim.data.qpos[7:10].copy()
        normal_cubeB_pos = env.sim.data.qpos[14:17].copy()
        normal_robot_pos = env.sim.data.qpos[0:7].copy()
        print(f"   cubeA pos after normal reset: {normal_cubeA_pos}")
        print(f"   cubeB pos after normal reset: {normal_cubeB_pos}")
        print(f"   robot joints after normal reset: {normal_robot_pos}")

        # Step 2: Deterministic reset (prevents randomization)
        print("\n2. Deterministic reset (no randomization):")
        original_deterministic = env.deterministic_reset
        env.deterministic_reset = True
        env.reset()
        env.deterministic_reset = original_deterministic
        deterministic_cubeA_pos = env.sim.data.qpos[7:10].copy()
        deterministic_cubeB_pos = env.sim.data.qpos[14:17].copy()
        deterministic_robot_pos = env.sim.data.qpos[0:7].copy()
        print(f"   cubeA pos after deterministic reset: {deterministic_cubeA_pos}")
        print(f"   cubeB pos after deterministic reset: {deterministic_cubeB_pos}")
        print(f"   robot joints after deterministic reset: {deterministic_robot_pos}")

        # Step 3: Apply comprehensive state restoration (exactly like playback)
        print("\n3. Applying comprehensive state restoration:")
        env.sim.data.qpos[:] = qpos
        env.sim.data.qvel[:] = qvel
        env.sim.forward()

        restored_cubeA_pos = env.sim.data.qpos[7:10].copy()
        restored_cubeB_pos = env.sim.data.qpos[14:17].copy()
        restored_robot_pos = env.sim.data.qpos[0:7].copy()
        print(f"   cubeA pos after restoration: {restored_cubeA_pos}")
        print(f"   cubeB pos after restoration: {restored_cubeB_pos}")
        print(f"   robot joints after restoration: {restored_robot_pos}")

        # Compare with saved
        cubeA_diff = np.linalg.norm(restored_cubeA_pos - qpos[7:10])
        cubeB_diff = np.linalg.norm(restored_cubeB_pos - qpos[14:17])
        robot_diff = np.linalg.norm(restored_robot_pos - qpos[0:7])

        print("\n4. Comparison with saved state:")
        print(f"   cubeA difference: {cubeA_diff:.2e}")
        print(f"   cubeB difference: {cubeB_diff:.2e}")
        print(f"   robot difference: {robot_diff:.2e}")

        if cubeA_diff < 1e-10 and cubeB_diff < 1e-10 and robot_diff < 1e-10:
            print("   ✓✓✓ PERFECT STATE RESTORATION ✓✓✓")
        else:
            print("   ✗ RESTORATION FAILED")
            print(f"   Expected cubeA: {qpos[7:10]}")
            print(f"   Expected cubeB: {qpos[14:17]}")
            print(f"   Expected robot: {qpos[0:7]}")

    else:
        print("\n✗ No comprehensive state data found")
        print("This demo was recorded before the fix was implemented")

    # Check fallback data
    print("\n" + "-" * 50)
    print("CHECKING FALLBACK DATA")
    print("-" * 50)

    if 'cubeA_joint_qpos' in f:
        fallback_cubeA = f['cubeA_joint_qpos'][:]
        print(f"Fallback cubeA qpos: {fallback_cubeA}")

    if 'cubeB_joint_qpos' in f:
        fallback_cubeB = f['cubeB_joint_qpos'][:]
        print(f"Fallback cubeB qpos: {fallback_cubeB}")

    if 'robot0_joint_pos' in f:
        fallback_robot = f['robot0_joint_pos'][:]
        print(f"Fallback robot qpos: {fallback_robot}")

print("\n" + "=" * 80)
print("DIAGNOSIS")
print("=" * 80)

if 'initial_state_data' in f:
    print("✓ This demo has comprehensive state data")
    print("✓ Playback should restore exact positions")
    print("⚠ If objects still appear wrong, there may be a rendering issue")
    print("💡 Try running playback with has_renderer=True to see visually")
else:
    print("✗ This demo lacks comprehensive state data")
    print("⚠ Recorded before the fix - using fallback methods")
    print("💡 Record new demos to get perfect restoration")

print("=" * 80)

env.close()