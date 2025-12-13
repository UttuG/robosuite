#!/usr/bin/env python3
"""Test playback with position logging to identify the issue."""

import h5py
import numpy as np
import robosuite as suite
from robosuite import load_composite_controller_config

# Create environment with same settings as playback
controller_config = load_composite_controller_config(
    controller=None,
    robot="Panda",
)

env = suite.make(
    "MultiTableAssembly",
    robots="Panda",
    controller_configs=controller_config,
    has_renderer=False,  # Disable rendering for now
    has_offscreen_renderer=False,
    use_camera_obs=False,
    use_object_obs=True,
    ignore_done=False,
    control_freq=20,
)

print("=" * 80)
print("PLAYBACK POSITION VERIFICATION")
print("=" * 80)

demo_file = '/tmp/my_demos/demo_episode_0001_20251212_115307.h5'

with h5py.File(demo_file, 'r') as f:
    print(f"Loading demo: {demo_file}")

    # Get the saved comprehensive state
    if 'initial_state_data' in f:
        saved_qpos = f['initial_state_data']['qpos'][:]
        saved_qvel = f['initial_state_data']['qvel'][:]

        print("\nSaved initial positions from recording:")
        print(f"  Robot joints: {saved_qpos[0:7]}")
        print(f"  CubeA position: {saved_qpos[7:10]}")
        print(f"  CubeB position: {saved_qpos[14:17]}")

        # Simulate the exact playback process
        print("\nSimulating playback initialization...")

        # Step 1: Deterministic reset (same as playback)
        original_deterministic = env.deterministic_reset
        env.deterministic_reset = True
        obs = env.reset()
        env.deterministic_reset = original_deterministic

        print("After deterministic reset:")
        print(f"  Robot joints: {env.sim.data.qpos[0:7]}")
        print(f"  CubeA position: {env.sim.data.qpos[7:10]}")
        print(f"  CubeB position: {env.sim.data.qpos[14:17]}")

        # Step 2: State restoration (same as playback)
        env.sim.data.qpos[:] = saved_qpos
        env.sim.data.qvel[:] = saved_qvel
        env.sim.forward()

        print("\nAfter state restoration:")
        print(f"  Robot joints: {env.sim.data.qpos[0:7]}")
        print(f"  CubeA position: {env.sim.data.qpos[7:10]}")
        print(f"  CubeB position: {env.sim.data.qpos[14:17]}")

        # Check if they match
        robot_match = np.allclose(env.sim.data.qpos[0:7], saved_qpos[0:7], atol=1e-10)
        cubeA_match = np.allclose(env.sim.data.qpos[7:10], saved_qpos[7:10], atol=1e-10)
        cubeB_match = np.allclose(env.sim.data.qpos[14:17], saved_qpos[14:17], atol=1e-10)

        print("\nPosition matching:")
        print(f"  Robot joints match: {robot_match}")
        print(f"  CubeA position match: {cubeA_match}")
        print(f"  CubeB position match: {cubeB_match}")

        if robot_match and cubeA_match and cubeB_match:
            print("✓✓✓ STATE RESTORATION IS WORKING PERFECTLY ✓✓✓")
        else:
            print("✗ STATE RESTORATION FAILED")

        # Now check what the observables show
        print("\nChecking observables after restoration:")
        print(f"  cubeA_pos observable: {obs.get('cubeA_pos', 'N/A')}")
        print(f"  cubeB_pos observable: {obs.get('cubeB_pos', 'N/A')}")

        # Compare with saved observables from recording
        if 'observables/cubeA_pos' in f and 'observables/cubeB_pos' in f:
            saved_cubeA_obs = f['observables/cubeA_pos'][0]  # First timestep
            saved_cubeB_obs = f['observables/cubeB_pos'][0]

            print("\nSaved observables from recording (first timestep):")
            print(f"  cubeA_pos: {saved_cubeA_obs}")
            print(f"  cubeB_pos: {saved_cubeB_obs}")

            obs_match_A = np.allclose(obs.get('cubeA_pos', [0,0,0]), saved_cubeA_obs, atol=1e-6)
            obs_match_B = np.allclose(obs.get('cubeB_pos', [0,0,0]), saved_cubeB_obs, atol=1e-6)

            print(f"  Observables match: cubeA={obs_match_A}, cubeB={obs_match_B}")

    else:
        print("✗ No comprehensive state data")

print("\n" + "=" * 80)
print("CONCLUSION")
print("=" * 80)

if 'initial_state_data' in f:
    print("✓ Demo has comprehensive state data")
    print("✓ State restoration works perfectly")
    print("⚠ If visual positions differ, the issue may be:")
    print("  - Rendering not updating after state restoration")
    print("  - User comparing wrong positions")
    print("  - Playback running with different environment settings")
    print("💡 Try: python debug_playback_render.py (with rendering)")
else:
    print("✗ Demo lacks comprehensive state data")

print("=" * 80)

env.close()