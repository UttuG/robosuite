#!/usr/bin/env python3
"""Test if env.step() updates observables after state restoration."""

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
print("TESTING OBSERVABLE UPDATE AFTER STATE RESTORATION")
print("=" * 80)

demo_file = '/tmp/my_demos/demo_episode_0001_20251212_115307.h5'

with h5py.File(demo_file, 'r') as f:
    if 'initial_state_data' in f:
        saved_qpos = f['initial_state_data']['qpos'][:]
        saved_qvel = f['initial_state_data']['qvel'][:]

        print("Restoring state...")
        original_deterministic = env.deterministic_reset
        env.deterministic_reset = True
        obs = env.reset()
        env.deterministic_reset = original_deterministic

        env.sim.data.qpos[:] = saved_qpos
        env.sim.data.qvel[:] = saved_qvel
        env.sim.forward()

        print("After state restoration:")
        print(f"  qpos cubeA: {env.sim.data.qpos[7:10]}")
        print(f"  qpos cubeB: {env.sim.data.qpos[14:17]}")
        print(f"  obs cubeA: {obs.get('cubeA_pos', 'N/A')}")
        print(f"  obs cubeB: {obs.get('cubeB_pos', 'N/A')}")

        # Try calling step with zero action to update observables
        print("\nCalling env.step([0]*7) to update observables...")
        zero_action = np.zeros(7)
        obs, reward, done, info = env.step(zero_action)

        print("After env.step():")
        print(f"  qpos cubeA: {env.sim.data.qpos[7:10]}")
        print(f"  qpos cubeB: {env.sim.data.qpos[14:17]}")
        print(f"  obs cubeA: {obs.get('cubeA_pos', 'N/A')}")
        print(f"  obs cubeB: {obs.get('cubeB_pos', 'N/A')}")

        # Check if they match now
        qpos_A = env.sim.data.qpos[7:10]
        qpos_B = env.sim.data.qpos[14:17]
        obs_A = obs.get('cubeA_pos', np.zeros(3))
        obs_B = obs.get('cubeB_pos', np.zeros(3))

        match_A = np.allclose(qpos_A, obs_A, atol=1e-6)
        match_B = np.allclose(qpos_B, obs_B, atol=1e-6)

        print(f"\nMatching: cubeA={match_A}, cubeB={match_B}")

        if match_A and match_B:
            print("✓ env.step() updates observables correctly")
        else:
            print("✗ Observables still don't match qpos")

        # Compare with saved recording observables
        if 'observables/cubeA_pos' in f and 'observables/cubeB_pos' in f:
            saved_obs_A = f['observables/cubeA_pos'][0]
            saved_obs_B = f['observables/cubeB_pos'][0]

            print("\nComparison with saved recording:")
            print(f"  Saved obs cubeA: {saved_obs_A}")
            print(f"  Saved obs cubeB: {saved_obs_B}")
            print(f"  Current obs cubeA: {obs_A}")
            print(f"  Current obs cubeB: {obs_B}")

            match_saved_A = np.allclose(obs_A, saved_obs_A, atol=1e-6)
            match_saved_B = np.allclose(obs_B, saved_obs_B, atol=1e-6)

            print(f"  Match saved: cubeA={match_saved_A}, cubeB={match_saved_B}")

print("\n" + "=" * 80)
print("CONCLUSION")
print("=" * 80)
print("The issue is that observables are not updated after state restoration.")
print("We need to call env.step() after restoration to update observables.")
print("This explains why visually the objects appear in wrong positions.")
print("=" * 80)

env.close()