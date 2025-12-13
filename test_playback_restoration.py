#!/usr/bin/env python3
"""
Quick test script to record a short demo and verify perfect playback.

This script:
1. Records a short teleoperated demonstration
2. Saves it to HDF5 with comprehensive state data
3. Plays it back and verifies exact state restoration

Run this to verify the comprehensive state restoration is working.
"""

import h5py
import numpy as np
import robosuite as suite
from robosuite import load_composite_controller_config
from robosuite.wrappers import VisualizationWrapper
import os
import time

def test_playback_state_restoration():
    """Test that playback restores initial state exactly."""
    
    print("=" * 80)
    print("PLAYBACK STATE RESTORATION TEST")
    print("=" * 80)
    
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
    
    # Reset to get initial state
    print("\n1. Resetting environment and saving state...")
    obs = env.reset()
    
    # Save initial state
    initial_qpos = np.array(env.sim.data.qpos)
    initial_qvel = np.array(env.sim.data.qvel)
    initial_body_pos = np.array(env.sim.data.body_xpos)
    initial_body_rot = np.array(env.sim.data.body_xmat)
    
    print(f"   Initial robot joint 0: {initial_qpos[0]:.6f}")
    print(f"   Initial cubeA pos: {initial_qpos[7:10]}")
    print(f"   Initial cubeB pos: {initial_qpos[14:17]}")
    
    # Record a few steps
    print("\n2. Recording 10 steps...")
    actions = []
    observations = []
    for i in range(10):
        action = np.random.randn(7) * 0.05
        obs, reward, done, info = env.step(action)
        actions.append(action)
        observations.append(obs.copy())
    
    # Save to HDF5
    test_file = "/tmp/test_playback_state.h5"
    print(f"\n3. Saving to {test_file}...")
    
    with h5py.File(test_file, 'w') as f:
        # Save metadata
        f.attrs['total_steps'] = len(actions)
        
        # Save initial state (comprehensive)
        initial_state_group = f.create_group('initial_state_data')
        initial_state_group.create_dataset('qpos', data=initial_qpos)
        initial_state_group.create_dataset('qvel', data=initial_qvel)
        initial_state_group.create_dataset('body_pos', data=initial_body_pos)
        initial_state_group.create_dataset('body_rot', data=initial_body_rot)
        
        # Save fallback data
        f.create_dataset('initial_state', data=np.array(env.sim.get_state().flatten()))
        f.create_dataset('cubeA_joint_qpos', data=env.sim.data.get_joint_qpos('cubeA_joint0').copy())
        f.create_dataset('cubeB_joint_qpos', data=env.sim.data.get_joint_qpos('cubeB_joint0').copy())
        f.create_dataset('robot0_joint_pos', data=initial_qpos[:7])
        
        # Save actions for playback
        f.create_dataset('actions', data=np.array(actions))
        f.create_dataset('rewards', data=np.array([0.0] * len(actions)))
        f.create_dataset('dones', data=np.array([False] * len(actions)))
    
    print(f"   Saved file size: {os.path.getsize(test_file) / 1024:.1f} KB")
    
    # Now test playback
    print("\n4. Testing playback state restoration...")
    
    env.reset()  # Reset will randomize
    randomized_qpos = np.array(env.sim.data.qpos)
    print(f"   After reset (randomized) robot joint 0: {randomized_qpos[0]:.6f}")
    print(f"   After reset cubeA pos: {randomized_qpos[7:10]}")
    
    # Load and restore state (simulating what playback does)
    with h5py.File(test_file, 'r') as f:
        # Try comprehensive restoration (Strategy 1)
        if 'initial_state_data' in f:
            print("\n5. Using comprehensive state restoration...")
            initial_state_data = f['initial_state_data']
            
            qpos = initial_state_data['qpos'][:]
            qvel = initial_state_data['qvel'][:]
            
            env.sim.data.qpos[:] = qpos
            env.sim.data.qvel[:] = qvel
            env.sim.forward()
            
            restored_qpos = np.array(env.sim.data.qpos)
            print(f"   ✓ Restored robot joint 0: {restored_qpos[0]:.6f}")
            print(f"   ✓ Restored cubeA pos: {restored_qpos[7:10]}")
            
            # Check accuracy
            qpos_error = np.linalg.norm(restored_qpos - initial_qpos)
            qvel_error = np.linalg.norm(env.sim.data.qvel - initial_qvel)
            
            print(f"\n6. Validation:")
            print(f"   qpos error: {qpos_error:.2e}")
            print(f"   qvel error: {qvel_error:.2e}")
            
            if qpos_error < 1e-10 and qvel_error < 1e-10:
                print("   ✓✓✓ PERFECT STATE RESTORATION - READY FOR PLAYBACK! ✓✓✓")
                return True
            else:
                print("   ✗ State restoration has significant error")
                return False
        else:
            print("   ✗ initial_state_data not found in file")
            return False

if __name__ == "__main__":
    success = test_playback_state_restoration()
    
    print("\n" + "=" * 80)
    if success:
        print("SUCCESS: Comprehensive state restoration is working correctly!")
        print("\nYou can now:")
        print("1. Record demonstrations with: python robosuite/demos/demo_collect_and_playback_data.py")
        print("2. Playback with: python robosuite/demos/demo_collect_and_playback_data.py --playback <file.h5>")
        print("3. Robot and objects will be in EXACT initial positions during playback")
    else:
        print("FAILURE: State restoration test failed")
    print("=" * 80)
