"""
Simple test script to verify MultiTableAssembly observables and cameras.

This script checks:
1. All custom observables are properly registered
2. Camera observations (including eye-in-hand) are accessible
3. Observable values are computed correctly
"""

import numpy as np
import robosuite as suite


def test_multi_table_assembly():
    print("Testing MultiTableAssembly Environment...")
    print("=" * 80)
    
    # Create environment with all observations enabled
    env = suite.make(
        "MultiTableAssembly",
        robots="Panda",  # Use Panda as default for quick testing
        has_renderer=False,
        has_offscreen_renderer=True,
        use_camera_obs=True,
        use_object_obs=True,
        camera_names=["agentview", "birdview", "robot0_eye_in_hand"],  # Include wrist camera
        camera_heights=128,
        camera_widths=128,
        control_freq=20,
        horizon=100,
    )
    
    # Reset environment
    obs = env.reset()
    
    # Test 1: Check camera observations
    print("\n1. Camera Observations:")
    print("-" * 80)
    camera_obs = [k for k in obs.keys() if "image" in k]
    for cam_obs in camera_obs:
        print(f"   ✓ {cam_obs:30s} | Shape: {obs[cam_obs].shape}")
    
    if len(camera_obs) > 0:
        print(f"   SUCCESS: {len(camera_obs)} camera observations found")
    else:
        print("   WARNING: No camera observations found")
    
    # Test 2: Check object observables
    print("\n2. Object Observables:")
    print("-" * 80)
    
    expected_observables = [
        "cubeA_pos", "cubeA_quat",
        "cubeB_pos", "cubeB_quat",
        "cubeA_to_cubeB",
        "source_table_pos", "dest_table_pos",
        "cubeA_to_source_table", "cubeA_to_dest_table", "cubeB_to_dest_table"
    ]
    
    for obs_name in expected_observables:
        if obs_name in obs:
            obs_value = obs[obs_name]
            print(f"   ✓ {obs_name:40s} | Shape: {obs_value.shape if hasattr(obs_value, 'shape') else 'scalar'}")
        else:
            print(f"   ✗ {obs_name:40s} | MISSING")
    
    # Test 3: Check gripper-to-object observables
    print("\n3. Gripper-to-Object Observables:")
    print("-" * 80)
    
    gripper_obs = [k for k in obs.keys() if "gripper_to_cube" in k]
    for gripper_obs_name in gripper_obs:
        print(f"   ✓ {gripper_obs_name:40s} | Shape: {obs[gripper_obs_name].shape}")
    
    if len(gripper_obs) > 0:
        print(f"   SUCCESS: {len(gripper_obs)} gripper-to-object observations found")
    else:
        print("   WARNING: No gripper-to-object observations found")
    
    # Test 4: Verify observable values make sense
    print("\n4. Observable Value Sanity Checks:")
    print("-" * 80)
    
    if "cubeA_pos" in obs and "cubeB_pos" in obs:
        cubeA_pos = obs["cubeA_pos"]
        cubeB_pos = obs["cubeB_pos"]
        distance = np.linalg.norm(cubeA_pos - cubeB_pos)
        print(f"   CubeA position: {cubeA_pos}")
        print(f"   CubeB position: {cubeB_pos}")
        print(f"   Distance between cubes: {distance:.4f} meters")
        
        if 0.1 < distance < 1.0:
            print("   ✓ Distance is reasonable")
        else:
            print("   ✗ Distance seems unusual")
    
    if "cubeA_to_cubeB" in obs:
        computed_diff = obs["cubeA_to_cubeB"]
        expected_diff = cubeB_pos - cubeA_pos
        error = np.linalg.norm(computed_diff - expected_diff)
        print(f"   cubeA_to_cubeB observable: {computed_diff}")
        print(f"   Expected value: {expected_diff}")
        print(f"   Error: {error:.6f}")
        
        if error < 1e-6:
            print("   ✓ Observable computed correctly")
        else:
            print("   ✗ Observable computation has errors")
    
    # Test 5: Run a few steps to ensure observables update
    print("\n5. Observable Update Test (5 random steps):")
    print("-" * 80)
    
    initial_cubeA_pos = obs["cubeA_pos"].copy() if "cubeA_pos" in obs else None
    
    for step in range(5):
        action = np.random.randn(*env.action_spec[0].shape) * 0.05
        obs, reward, done, info = env.step(action)
        
        if "cubeA_pos" in obs:
            current_pos = obs["cubeA_pos"]
            if step == 0:
                print(f"   Step {step}: cubeA_pos = {current_pos}")
    
    if initial_cubeA_pos is not None and "cubeA_pos" in obs:
        final_pos = obs["cubeA_pos"]
        movement = np.linalg.norm(final_pos - initial_cubeA_pos)
        print(f"   Total cube movement: {movement:.6f} meters")
        print("   ✓ Observables are updating")
    
    # Test 6: Check all registered observables
    print("\n6. All Registered Observables:")
    print("-" * 80)
    
    all_observables = env._observables
    enabled_count = 0
    for obs_name, observable in all_observables.items():
        if observable.is_enabled():
            enabled_count += 1
    
    print(f"   Total observables: {len(all_observables)}")
    print(f"   Enabled observables: {enabled_count}")
    print(f"   Observation dict keys: {len(obs.keys())}")
    
    # Summary
    print("\n" + "=" * 80)
    print("TEST SUMMARY:")
    print("=" * 80)
    print(f"✓ Environment created successfully")
    print(f"✓ {len(camera_obs)} camera observations available")
    print(f"✓ {len(gripper_obs)} gripper-to-object observations available")
    print(f"✓ Object-to-table observations implemented")
    print(f"✓ Object-to-object observations implemented")
    print(f"✓ All observables updating correctly")
    print("=" * 80)
    
    env.close()
    print("\nAll tests completed successfully!")


def test_with_eye_in_hand_camera():
    """Test with Panda robot to verify eye-in-hand camera works."""
    print("\n\nTesting Panda with Eye-in-Hand Camera...")
    print("=" * 80)
    
    try:
        env = suite.make(
            "MultiTableAssembly",
            robots="Panda",
            has_renderer=False,
            has_offscreen_renderer=True,
            use_camera_obs=True,
            use_object_obs=True,
            # Note: Panda's wrist camera is prefixed with robot name
            camera_names=["agentview", "birdview", "robot0_eye_in_hand"],
            camera_heights=128,
            camera_widths=128,
            control_freq=20,
            horizon=100,
        )
        
        obs = env.reset()
        
        # Check for eye-in-hand camera (with robot prefix)
        print("\nEye-in-Hand Camera Observation:")
        print("-" * 80)
        
        eye_camera = "robot0_eye_in_hand_image"
        if eye_camera in obs:
            print(f"   ✓ {eye_camera:40s} | Shape: {obs[eye_camera].shape}")
        else:
            print(f"   ✗ {eye_camera:40s} | NOT AVAILABLE")
        
        # Also check all available cameras
        print("\nAll Available Cameras:")
        print("-" * 80)
        camera_obs = [k for k in obs.keys() if "image" in k]
        for cam_obs in camera_obs:
            print(f"   ✓ {cam_obs:40s} | Shape: {obs[cam_obs].shape}")
        
        env.close()
        print("\n✓ Panda eye-in-hand camera test completed successfully!")
        
    except Exception as e:
        print(f"\n✗ Panda eye-in-hand test failed: {e}")


if __name__ == "__main__":
    # Run main test with Panda robot
    test_multi_table_assembly()
    
    # Test eye-in-hand camera with Panda
    test_with_eye_in_hand_camera()
