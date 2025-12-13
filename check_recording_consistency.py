#!/usr/bin/env python3
"""Check if the recording process itself captures the correct initial state."""

import numpy as np
import robosuite as suite
from robosuite import load_composite_controller_config

# Create environment exactly like in teleoperation
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
print("CHECKING RECORDING INITIAL STATE CAPTURE")
print("=" * 80)

# Simulate what happens during recording
print("\n1. Fresh environment reset (like start of recording):")
obs = env.reset()
print(f"   qpos cubeA: {env.sim.data.qpos[7:10]}")
print(f"   qpos cubeB: {env.sim.data.qpos[14:17]}")
print(f"   obs cubeA: {obs.get('cubeA_pos', 'N/A')}")
print(f"   obs cubeB: {obs.get('cubeB_pos', 'N/A')}")

# This is what gets saved as initial_state_data during recording
saved_qpos = env.sim.data.qpos.copy()
saved_qvel = env.sim.data.qvel.copy()

print("\n2. Saved initial state (what gets stored in HDF5):")
print(f"   saved qpos cubeA: {saved_qpos[7:10]}")
print(f"   saved qpos cubeB: {saved_qpos[14:17]}")

# Now simulate playback restoration
print("\n3. Playback restoration simulation:")
env2 = suite.make(
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

# Deterministic reset
original_deterministic = env2.deterministic_reset
env2.deterministic_reset = True
obs2 = env2.reset()
env2.deterministic_reset = original_deterministic

# State restoration
env2.sim.data.qpos[:] = saved_qpos
env2.sim.data.qvel[:] = saved_qvel
env2.sim.forward()

# Update observables
zero_action = np.zeros(env2.action_dim)
obs2, _, _, _ = env2.step(zero_action)

print(f"   restored qpos cubeA: {env2.sim.data.qpos[7:10]}")
print(f"   restored qpos cubeB: {env2.sim.data.qpos[14:17]}")
print(f"   restored obs cubeA: {obs2.get('cubeA_pos', 'N/A')}")
print(f"   restored obs cubeB: {obs2.get('cubeB_pos', 'N/A')}")

# Check if they match
qpos_match_A = np.allclose(env.sim.data.qpos[7:10], env2.sim.data.qpos[7:10])
qpos_match_B = np.allclose(env.sim.data.qpos[14:17], env2.sim.data.qpos[14:17])
obs_match_A = np.allclose(obs.get('cubeA_pos', [0,0,0]), obs2.get('cubeA_pos', [0,0,0]))
obs_match_B = np.allclose(obs.get('cubeB_pos', [0,0,0]), obs2.get('cubeB_pos', [0,0,0]))

print("\n4. Comparison:")
print(f"   qpos cubeA match: {qpos_match_A}")
print(f"   qpos cubeB match: {qpos_match_B}")
print(f"   obs cubeA match: {obs_match_A}")
print(f"   obs cubeB match: {obs_match_B}")

if qpos_match_A and qpos_match_B and obs_match_A and obs_match_B:
    print("✓✓✓ RECORDING AND PLAYBACK ARE CONSISTENT ✓✓✓")
else:
    print("✗ INCONSISTENCY DETECTED")
    print("   This suggests the environment has non-deterministic initialization")

print("\n" + "=" * 80)
print("DIAGNOSIS")
print("=" * 80)
print("If objects appear in different positions, it could be:")
print("1. Environment has randomization that affects object placement")
print("2. User is comparing positions at different points in time")
print("3. Recording and playback use different environment seeds")
print("4. Objects move during teleoperation before state is saved")
print("=" * 80)

env.close()
env2.close()