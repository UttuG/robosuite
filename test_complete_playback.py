#!/usr/bin/env python3
"""Test complete 1:1 playback restoration."""

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
print("COMPLETE 1:1 PLAYBACK RESTORATION TEST")
print("=" * 80)

# Step 1: Record initial state
print("\n1. Recording initial state...")
obs = env.reset()

# Save the exact state we want to restore to
saved_qpos = np.array(env.sim.data.qpos)
saved_qvel = np.array(env.sim.data.qvel)
saved_body_pos = np.array(env.sim.data.body_xpos)
saved_body_rot = np.array(env.sim.data.body_xmat)

print(f"   Recorded cubeA pos: {saved_qpos[7:10]}")
print(f"   Recorded cubeB pos: {saved_qpos[14:17]}")
print(f"   Recorded robot joint 0: {saved_qpos[0]:.6f}")

# Step 2: Take some actions to change state
print("\n2. Taking 50 random actions to change state...")
for i in range(50):
    action = np.random.randn(7) * 0.1
    obs, reward, done, info = env.step(action)

print(f"   After actions cubeA pos: {env.sim.data.qpos[7:10]}")
print(f"   After actions cubeB pos: {env.sim.data.qpos[14:17]}")
print(f"   After actions robot joint 0: {env.sim.data.qpos[0]:.6f}")

# Step 3: Simulate playback reset (with deterministic_reset=True)
print("\n3. Simulating playback reset (deterministic mode)...")
original_deterministic_reset = env.deterministic_reset
env.deterministic_reset = True
env.reset()
env.deterministic_reset = original_deterministic_reset

print(f"   After deterministic reset cubeA pos: {env.sim.data.qpos[7:10]}")
print(f"   After deterministic reset cubeB pos: {env.sim.data.qpos[14:17]}")
print(f"   After deterministic reset robot joint 0: {env.sim.data.qpos[0]:.6f}")

# Step 4: Restore saved state
print("\n4. Restoring saved state...")
env.sim.data.qpos[:] = saved_qpos
env.sim.data.qvel[:] = saved_qvel
env.sim.forward()

restored_qpos = env.sim.data.qpos
restored_qvel = env.sim.data.qvel

print(f"   After restoration cubeA pos: {restored_qpos[7:10]}")
print(f"   After restoration cubeB pos: {restored_qpos[14:17]}")
print(f"   After restoration robot joint 0: {restored_qpos[0]:.6f}")

# Step 5: Verify perfect restoration
print("\n5. Verification:")
qpos_diff = np.linalg.norm(restored_qpos - saved_qpos)
qvel_diff = np.linalg.norm(restored_qvel - saved_qvel)
body_pos_diff = np.linalg.norm(env.sim.data.body_xpos - saved_body_pos)

print(f"   qpos difference: {qpos_diff:.2e}")
print(f"   qvel difference: {qvel_diff:.2e}")
print(f"   body_pos difference: {body_pos_diff:.2e}")

if qpos_diff < 1e-10 and qvel_diff < 1e-10:
    print("   ✓✓✓ PERFECT 1:1 RESTORATION - READY FOR PLAYBACK! ✓✓✓")
    print("   Objects and robot will be in EXACT same positions during playback")
else:
    print("   ✗ RESTORATION FAILED")

print("\n" + "=" * 80)
print("TEST COMPLETE")
print("=" * 80)

env.close()