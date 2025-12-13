#!/usr/bin/env python3
"""Test comprehensive state saving and restoration."""

import h5py
import numpy as np
import robosuite as suite
from robosuite import load_composite_controller_config
from robosuite.wrappers import VisualizationWrapper

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
print("COMPREHENSIVE STATE SAVE/RESTORE TEST")
print("=" * 80)

# Reset environment
obs = env.reset()

# Save initial state comprehensively
print("\n1. Saving initial state...")
initial_qpos = np.array(env.sim.data.qpos)
initial_qvel = np.array(env.sim.data.qvel)
initial_body_pos = np.array(env.sim.data.body_xpos)
initial_state = np.array(env.sim.get_state().flatten())

print(f"   Saved qpos shape: {initial_qpos.shape}")
print(f"   Saved qvel shape: {initial_qvel.shape}")
print(f"   Saved state shape: {initial_state.shape}")
print(f"   CubeA pos: {initial_qpos[7:10]}")
print(f"   CubeB pos: {initial_qpos[14:17]}")
print(f"   Robot joint 0: {initial_qpos[0]:.6f}")

# Take a few steps
print("\n2. Taking 20 steps with random actions...")
for i in range(20):
    # Get the first robot and create a small action
    action = np.zeros(7)  # Panda has 7 DOF
    action += np.random.randn(7) * 0.05
    obs, reward, done, info = env.step(action)

# Check current state differs
print(f"\n3. Current state after 20 steps:")
current_qpos = env.sim.data.qpos
print(f"   CubeA pos: {current_qpos[7:10]}")
print(f"   CubeB pos: {current_qpos[14:17]}")
print(f"   Robot joint 0: {current_qpos[0]:.6f}")

# Reset environment
print("\n4. Resetting environment (will randomize)...")
env.reset()
reset_qpos = env.sim.data.qpos
print(f"   CubeA pos after reset: {reset_qpos[7:10]}")
print(f"   CubeB pos after reset: {reset_qpos[14:17]}")
print(f"   Robot joint 0: {reset_qpos[0]:.6f}")

# Now restore comprehensive state
print("\n5. Restoring comprehensive state...")
env.sim.data.qpos[:] = initial_qpos
env.sim.data.qvel[:] = initial_qvel
env.sim.forward()

restored_qpos = env.sim.data.qpos
print(f"   CubeA pos after restore: {restored_qpos[7:10]}")
print(f"   CubeB pos after restore: {restored_qpos[14:17]}")
print(f"   Robot joint 0: {restored_qpos[0]:.6f}")

# Check if restoration was successful
print("\n6. Checking restoration accuracy...")
qpos_diff = np.linalg.norm(restored_qpos - initial_qpos)
qvel_diff = np.linalg.norm(env.sim.data.qvel - initial_qvel)
print(f"   qpos difference: {qpos_diff:.10f}")
print(f"   qvel difference: {qvel_diff:.10f}")

if qpos_diff < 1e-6 and qvel_diff < 1e-6:
    print("   ✓ Comprehensive state restoration SUCCESSFUL!")
else:
    print("   ✗ Comprehensive state restoration FAILED!")

# Now test saving to HDF5
print("\n7. Testing HDF5 save/load...")
test_file = "/tmp/test_state.h5"

with h5py.File(test_file, 'w') as f:
    f.create_dataset('qpos', data=initial_qpos)
    f.create_dataset('qvel', data=initial_qvel)
    f.create_dataset('body_pos', data=initial_body_pos)
    
print(f"   Saved to {test_file}")

# Verify we can load it
with h5py.File(test_file, 'r') as f:
    loaded_qpos = f['qpos'][:]
    loaded_qvel = f['qvel'][:]
    print(f"   Loaded qpos shape: {loaded_qpos.shape}")
    print(f"   Loaded qvel shape: {loaded_qvel.shape}")
    
    # Check if loaded data matches
    load_qpos_diff = np.linalg.norm(loaded_qpos - initial_qpos)
    load_qvel_diff = np.linalg.norm(loaded_qvel - initial_qvel)
    print(f"   qpos load difference: {load_qpos_diff:.10f}")
    print(f"   qvel load difference: {load_qvel_diff:.10f}")
    
    if load_qpos_diff < 1e-10 and load_qvel_diff < 1e-10:
        print("   ✓ HDF5 save/load SUCCESSFUL!")
    else:
        print("   ✗ HDF5 save/load FAILED!")

print("\n" + "=" * 80)
print("TEST COMPLETE")
print("=" * 80)

env.close()
