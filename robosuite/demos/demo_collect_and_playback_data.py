"""
Teleoperate robot with keyboard and collect demonstration data in HDF5 format.

This script integrates keyboard teleoperation with data collection for the MultiTableAssembly environment.
During teleoperation, it records:
- RGB images from integrated cameras (agentview and robot0_eye_in_hand)
- Robot joint states and gripper status
- Relative poses and sensor readings (all observables from Stage 2)
- Control commands/actions applied during teleoperation

Data is stored in HDF5 format for efficient storage and later playback.

Keyboard Controls:
    Keys        Command
    Ctrl+q      reset simulation
    spacebar    toggle gripper (open/close)
    up-right-down-left    move horizontally in x-y plane
    . ;         move vertically
    o p         rotate (yaw)
    y h         rotate (pitch)
    e r         rotate (roll)
    b           toggle arm/base mode (if applicable)
    s           switch active arm (if multi-armed robot)
    =           switch active robot (if multi-robot environment)
"""

import argparse
import os
import time
from glob import glob

import h5py
import numpy as np

import robosuite as suite
from robosuite import load_composite_controller_config
from robosuite.controllers.composite.composite_controller import WholeBody
from robosuite.wrappers import VisualizationWrapper, DomainRandomizationWrapper


class HDF5DataCollectionWrapper:
    """
    HDF5-based data collection wrapper for recording teleoperation demonstrations.
    """

    def __init__(self, env, directory, collect_freq=1, flush_freq=100):
        """
        Initializes the HDF5 data collection wrapper.

        Args:
            env (MujocoEnv): The environment to monitor.
            directory (str): Where to store collected data.
            collect_freq (int): How often to save simulation state, in terms of environment steps.
            flush_freq (int): How frequently to dump data to disk, in terms of environment steps.
        """
        self.env = env
        self.directory = directory
        self.collect_freq = collect_freq
        self.flush_freq = flush_freq
        
        # Randomization info
        self.randomization_enabled = False
        self.randomization_seed = None

        # Create directory if it doesn't exist
        if not os.path.exists(self.directory):
            print(f"HDF5DataCollectionWrapper: making new directory at {self.directory}")
            os.makedirs(self.directory)

        # Episode tracking
        self.episode_count = 0
        self.current_episode_data = None
        self.episode_start_time = None
        self.episode_success = False

        # Current episode data buffers
        self.reset_episode_data()

    def set_randomization_info(self, enabled, seed):
        """Set randomization information to be saved with the episode."""
        self.randomization_enabled = enabled
        self.randomization_seed = seed

    def reset_episode_data(self):
        """Reset data buffers for new episode."""
        self.current_episode_data = {
            'actions': [],
            'rewards': [],
            'dones': [],
            'timestamps': [],
            'camera_images': {},
            'robot_states': [],
            'gripper_states': [],
            'observables': []
        }

        # Initialize camera image buffers
        camera_names = getattr(self.env.unwrapped if hasattr(self.env, 'unwrapped') else self.env, 'camera_names', [])
        if camera_names:
            for cam_name in camera_names:
                self.current_episode_data['camera_images'][cam_name] = []

    def start_episode(self):
        """Start a new episode."""
        self.episode_start_time = time.time()
        self.reset_episode_data()
        self.episode_success = False

        # Save EVERYTHING for complete state restoration
        # This includes model XML, full state, and all joint positions
        self._current_task_instance_xml = self.env.sim.model.get_xml()
        self._current_task_instance_state = np.array(self.env.sim.get_state().flatten())
        
        # Save all joint positions and velocities for all objects and robots
        self._all_joint_data = {
            'qpos': np.array(self.env.sim.data.qpos),  # All joint positions
            'qvel': np.array(self.env.sim.data.qvel),  # All joint velocities
            'body_pos': np.array(self.env.sim.data.body_xpos),  # All body positions
            'body_rot': np.array(self.env.sim.data.body_xmat),  # All body rotations
        }
        
        # Save specific object joint positions for explicit restoration
        self._cubeA_qpos = self.env.sim.data.get_joint_qpos('cubeA_joint0').copy()
        self._cubeB_qpos = self.env.sim.data.get_joint_qpos('cubeB_joint0').copy()
        
        # Save robot joint positions for explicit restoration
        self._robot_qpos = {}
        for i, robot in enumerate(self.env.robots):
            self._robot_qpos[f'robot{i}_joint_pos'] = robot._joint_positions.copy()
        
        self.episode_count += 1
        self.episode_success = False

    def record_step(self, observation, action, reward, done, info):
        """Record a single step of data."""
        timestamp = time.time() - self.episode_start_time

        # Update success status if achieved
        if info.get('success', False):
            self.episode_success = True

        # Record basic step data
        self.current_episode_data['actions'].append(action)
        self.current_episode_data['rewards'].append(reward)
        self.current_episode_data['dones'].append(done)
        self.current_episode_data['timestamps'].append(timestamp)

        # Record camera images if available
        camera_names = getattr(self.env, 'camera_names', [])
        if camera_names:
            for cam_name in camera_names:
                image_key = f"{cam_name}_image"
                if image_key in observation:
                    self.current_episode_data['camera_images'][cam_name].append(
                        observation[image_key].copy()
                    )

        # Record robot joint states
        robot_state = {}
        for i, robot in enumerate(self.env.robots):
            robot_state[f'robot{i}_joint_pos'] = robot._joint_positions.copy()
            robot_state[f'robot{i}_joint_vel'] = robot._joint_velocities.copy()
        self.current_episode_data['robot_states'].append(robot_state)

        # Record gripper states using proper accessor methods
        gripper_state = {}
        for i, robot in enumerate(self.env.robots):
            for arm in robot.arms:
                if robot.has_gripper[arm]:
                    # Access gripper joint positions from simulation
                    gripper_qpos = np.array([self.env.sim.data.qpos[x] for x in robot._ref_gripper_joint_pos_indexes[arm]])
                    gripper_qvel = np.array([self.env.sim.data.qvel[x] for x in robot._ref_gripper_joint_vel_indexes[arm]])
                    gripper_state[f'robot{i}_{arm}_gripper_qpos'] = gripper_qpos.copy()
                    gripper_state[f'robot{i}_{arm}_gripper_qvel'] = gripper_qvel.copy()
                    gripper_state[f'robot{i}_{arm}_gripper_action'] = robot.gripper[arm].current_action.copy()
        self.current_episode_data['gripper_states'].append(gripper_state)

        # Record all observables (including our custom ones)
        obs_data = {}
        for key, value in observation.items():
            if not key.endswith('_image'):  # Skip image data as it's handled separately
                if isinstance(value, np.ndarray):
                    obs_data[key] = value.copy()
                else:
                    obs_data[key] = value
        self.current_episode_data['observables'].append(obs_data)

    def save_episode(self):
        """Save current episode data to HDF5 file."""
        if not self.current_episode_data['actions']:
            return  # No data to save

        # Create filename with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"demo_episode_{self.episode_count:04d}_{timestamp}.h5"
        filepath = os.path.join(self.directory, filename)

        print(f"Saving episode {self.episode_count} to {filepath}")

        env = self.env.unwrapped if hasattr(self.env, 'unwrapped') else self.env

        with h5py.File(filepath, 'w') as f:
            # Save metadata
            f.attrs['episode_number'] = self.episode_count
            f.attrs['environment'] = self.env.__class__.__name__
            f.attrs['robots'] = str([robot.name for robot in self.env.robots])
            f.attrs['total_steps'] = len(self.current_episode_data['actions'])
            f.attrs['start_time'] = self.episode_start_time
            f.attrs['duration'] = time.time() - self.episode_start_time
            f.attrs['success'] = self.episode_success
            
            # Save randomization info
            f.attrs['randomization_enabled'] = self.randomization_enabled
            if self.randomization_seed is not None:
                f.attrs['randomization_seed'] = self.randomization_seed

            # Save model XML and initial state for playback
            f.create_dataset('model_xml', data=self._current_task_instance_xml, dtype=h5py.string_dtype())
            f.create_dataset('initial_state', data=self._current_task_instance_state)
            
            # Save ALL joint data comprehensively
            initial_state_group = f.create_group('initial_state_data')
            initial_state_group.create_dataset('qpos', data=self._all_joint_data['qpos'])
            initial_state_group.create_dataset('qvel', data=self._all_joint_data['qvel'])
            initial_state_group.create_dataset('body_pos', data=self._all_joint_data['body_pos'])
            initial_state_group.create_dataset('body_rot', data=self._all_joint_data['body_rot'])
            
            # Save object joint positions explicitly for fallback
            f.create_dataset('cubeA_joint_qpos', data=self._cubeA_qpos)
            f.create_dataset('cubeB_joint_qpos', data=self._cubeB_qpos)
            
            # Save robot joint positions explicitly for fallback
            for key, value in self._robot_qpos.items():
                f.create_dataset(key, data=value)

            # Save basic trajectory data
            # Use float64 for actions to minimize precision loss during playback
            f.create_dataset('actions', data=np.array(self.current_episode_data['actions']), dtype='f8')
            f.create_dataset('rewards', data=np.array(self.current_episode_data['rewards']))
            f.create_dataset('dones', data=np.array(self.current_episode_data['dones']))
            f.create_dataset('timestamps', data=np.array(self.current_episode_data['timestamps']))

            # Save camera images
            if self.current_episode_data['camera_images']:
                cam_group = f.create_group('camera_images')
                for cam_name, images in self.current_episode_data['camera_images'].items():
                    if images:  # Only save if we have images
                        cam_group.create_dataset(cam_name, data=np.array(images))

            # Save robot states
            robot_group = f.create_group('robot_states')
            if self.current_episode_data['robot_states']:
                # Save keys as metadata
                robot_group.attrs['keys'] = list(self.current_episode_data['robot_states'][0].keys())
                # Create dataset for each key
                for key in robot_group.attrs['keys']:
                    data = [step[key] for step in self.current_episode_data['robot_states']]
                    robot_group.create_dataset(key, data=np.array(data))

            # Save gripper states
            gripper_group = f.create_group('gripper_states')
            if self.current_episode_data['gripper_states']:
                gripper_group.attrs['keys'] = list(self.current_episode_data['gripper_states'][0].keys())
                for key in gripper_group.attrs['keys']:
                    data = [step[key] for step in self.current_episode_data['gripper_states']]
                    gripper_group.create_dataset(key, data=np.array(data))

            # Save observables
            obs_group = f.create_group('observables')
            if self.current_episode_data['observables']:
                obs_group.attrs['keys'] = list(self.current_episode_data['observables'][0].keys())
                for key in obs_group.attrs['keys']:
                    data = [step[key] for step in self.current_episode_data['observables']]
                    obs_group.create_dataset(key, data=np.array(data))

        success_marker = "✓ SUCCESS" if self.episode_success else "✗ INCOMPLETE"
        print(f"Episode {self.episode_count} saved with {len(self.current_episode_data['actions'])} steps [{success_marker}]")


def collect_teleoperation_trajectory(env, device, data_collector, max_fr=None, randomize=False, randomize_seed=None):
    """Collect trajectory data through keyboard teleoperation.

    Args:
        env (MujocoEnv): environment instance to collect trajectories from
        device: keyboard/spacemouse device for teleoperation
        data_collector: HDF5DataCollectionWrapper instance
        max_fr (int): if specified, pause the simulation whenever simulation runs faster than max_fr
        randomize (bool): whether to apply domain randomization
        randomize_seed (int): seed for domain randomization
    """
    
    # Apply domain randomization if requested
    if randomize:
        # Get all geom names
        all_geoms = env.sim.model.geom_names
        # Identify robot prefixes
        robot_prefixes = [robot.robot_model.naming_prefix for robot in env.robots]
        # Filter out robot geoms
        non_robot_geoms = [name for name in all_geoms if not any(name.startswith(prefix) for prefix in robot_prefixes)]
        
        # Use provided seed or generate random one
        if randomize_seed is None:
            randomize_seed = np.random.randint(0, 100000)
            
        print(f"Applying domain randomization with seed {randomize_seed}...")
        env = DomainRandomizationWrapper(
            env,
            seed=randomize_seed,
            randomize_color=True,
            randomize_camera=True,
            randomize_lighting=True,
            randomize_dynamics=False,
            color_randomization_args={
                'geom_names': non_robot_geoms, 
                'randomize_local': False, 
                'randomize_material': True
            },
            camera_randomization_args={
                'randomize_position': True, 
                'randomize_rotation': True, 
                'randomize_fovy': True
            },
            lighting_randomization_args={
                'randomize_position': True, 
                'randomize_direction': True, 
                'randomize_specular': True, 
                'randomize_ambient': True, 
                'randomize_diffuse': True
            },
            randomize_on_reset=True,
            randomize_every_n_steps=0
        )
        
        # Update data collector with randomization info
        data_collector.set_randomization_info(True, randomize_seed)
        # Update data collector's env reference since we wrapped it
        data_collector.env = env

    # Reset the environment
    obs = env.reset()

    # Start new episode (capture state AFTER reset to ensure we get the actual initial state)
    data_collector.start_episode()

    # Setup rendering
    env.render()

    # Initialize variables that should be maintained between resets
    last_grasp = 0

    # Initialize device control
    device.start_control()
    all_prev_gripper_actions = [
        {
            f"{robot_arm}_gripper": np.repeat([0], robot.gripper[robot_arm].dof)
            for robot_arm in robot.arms
            if robot.gripper[robot_arm].dof > 0
        }
        for robot in env.robots
    ]

    step_count = 0

    # Loop until we get a reset from the input or the task completes
    while True:
        start = time.time()

        # Set active robot
        active_robot = env.robots[device.active_robot]

        # Get the newest action
        input_ac_dict = device.input2action()

        # If action is none, then this a reset so we should break
        if input_ac_dict is None:
            break

        from copy import deepcopy

        action_dict = deepcopy(input_ac_dict)  # {}
        # set arm actions
        for arm in active_robot.arms:
            if isinstance(active_robot.composite_controller, WholeBody):  # input type passed to joint_action_policy
                controller_input_type = active_robot.composite_controller.joint_action_policy.input_type
            else:
                controller_input_type = active_robot.part_controllers[arm].input_type

            if controller_input_type == "delta":
                action_dict[arm] = input_ac_dict[f"{arm}_delta"]
            elif controller_input_type == "absolute":
                action_dict[arm] = input_ac_dict[f"{arm}_abs"]
            else:
                raise ValueError

        # Maintain gripper state for each robot but only update the active robot with action
        env_action = [robot.create_action_vector(all_prev_gripper_actions[i]) for i, robot in enumerate(env.robots)]
        env_action[device.active_robot] = active_robot.create_action_vector(action_dict)
        env_action = np.concatenate(env_action)
        for gripper_ac in all_prev_gripper_actions[device.active_robot]:
            all_prev_gripper_actions[device.active_robot][gripper_ac] = action_dict[gripper_ac]

        # Step environment
        obs, reward, done, info = env.step(env_action)

        # Check for task success
        success = env.unwrapped._check_success()
        info['success'] = success

        # Record the step
        data_collector.record_step(obs, env_action, reward, done, info)

        env.render()
        step_count += 1

        # Print status every 50 steps
        if step_count % 50 == 0:
            print(f"Step {step_count} | Reward: {reward:.3f} | Done: {done} | Success: {success}")

        # limit frame rate if necessary
        if max_fr is not None:
            elapsed = time.time() - start
            diff = 1 / max_fr - elapsed
            if diff > 0:
                time.sleep(diff)

        # Check if episode is done
        if done:
            print(f"\nHorizon reached at step {step_count}!")
            if success:
                print("✓ Task completed successfully!")
            break
        
        if success:
            print(f"\n✓ Task completed successfully at step {step_count}!")
            print("Episode will be marked as successful. Continue or press Ctrl+q to finish...")
            data_collector.episode_success = True

    # Save the episode data
    data_collector.save_episode()
    print(f"Episode completed with {step_count} steps")


def playback_trajectory(env, hdf5_file, max_fr=None, randomize=False, randomize_seed=None, record_dir=None):
    """Playback data from an HDF5 episode file with complete state restoration.

    Args:
        env (MujocoEnv): environment instance to playback trajectory in
        hdf5_file (str): Path to the HDF5 file containing episode data
        max_fr (int): if specified, pause the simulation whenever simulation runs faster than max_fr
        randomize (bool): whether to apply domain randomization during playback
        randomize_seed (int): seed for domain randomization
        record_dir (str): if specified, record the playback to this directory
    """

    print(f"Loading episode from {hdf5_file}")

    with h5py.File(hdf5_file, 'r') as f:
        actions = f['actions'][:]
        rewards = f['rewards'][:]
        dones = f['dones'][:]
        timestamps = f['timestamps'][:]
        
        # Check for recorded randomization
        recorded_randomize = f.attrs.get('randomization_enabled', False)
        recorded_seed = f.attrs.get('randomization_seed', None)

        print(f"Episode has {len(actions)} steps")
        print(f"Duration: {f.attrs.get('duration', 'N/A'):.2f} seconds")
        print(f"Environment: {f.attrs.get('environment', 'N/A')}")
        print(f"Robots: {f.attrs.get('robots', 'N/A')}")
        print(f"Success: {f.attrs.get('success', 'N/A')}")
        print(f"Recorded Randomization: {recorded_randomize} (Seed: {recorded_seed})")

        # Determine randomization strategy
        use_randomization = False
        seed_to_use = None
        
        if randomize:
            # User explicitly requested randomization (Data Augmentation)
            use_randomization = True
            seed_to_use = randomize_seed if randomize_seed is not None else np.random.randint(0, 100000)
            print(f"Applying NEW randomization (Data Augmentation) with seed {seed_to_use}")
        elif recorded_randomize:
            # Reproduce recorded randomization
            use_randomization = True
            seed_to_use = recorded_seed
            print(f"Reproducing RECORDED randomization with seed {seed_to_use}")
            
        # Apply randomization wrapper if needed
        if use_randomization:
            # Get all geom names
            all_geoms = env.sim.model.geom_names
            # Identify robot prefixes
            robot_prefixes = [robot.robot_model.naming_prefix for robot in env.robots]
            # Filter out robot geoms
            non_robot_geoms = [name for name in all_geoms if not any(name.startswith(prefix) for prefix in robot_prefixes)]
            # Also filter out gripper geoms (usually start with gripper prefix but not robot prefix)
            non_robot_geoms = [name for name in non_robot_geoms if "gripper" not in name]
            
            # Split into background and objects
            background_keywords = ['floor', 'wall']
            background_geoms = [name for name in non_robot_geoms if any(k in name for k in background_keywords)]
            
            # Objects: only cubes and cones (exclude tables, robot base, etc.)
            object_keywords = ['cube', 'cone']
            object_geoms = [name for name in non_robot_geoms if any(k in name for k in object_keywords)]
            
            print(f"Randomizing {len(background_geoms)} background geoms (walls, floors)")
            print(f"Randomizing {len(object_geoms)} object geoms (cubes/cones only)")
            
            # Wrapper 1: Background (Standard randomization + Camera/Lighting)
            if background_geoms:
                env = DomainRandomizationWrapper(
                    env,
                    seed=seed_to_use,
                    randomize_color=True,
                    randomize_camera=True,
                    randomize_lighting=True,
                    randomize_dynamics=False,
                    color_randomization_args={
                        'geom_names': background_geoms, 
                        'randomize_local': False, 
                        'randomize_material': True,
                        'randomize_skybox': False
                    },
                    camera_randomization_args={
                        'randomize_position': True, 
                        'randomize_rotation': True, 
                        'randomize_fovy': True
                    },
                    lighting_randomization_args={
                        'randomize_position': True, 
                        'randomize_direction': True, 
                        'randomize_specular': True, 
                        'randomize_ambient': True, 
                        'randomize_diffuse': True
                    },
                    randomize_on_reset=True,
                    randomize_every_n_steps=0
                )
                
            # Wrapper 2: Objects (Explicit RGB randomization only)
            # REMOVED: We will do this manually after reset to ensure it works
            # if object_geoms:
            #    env = DomainRandomizationWrapper(...)


        # Setup data collector if recording is requested
        data_collector = None
        if record_dir:
            data_collector = HDF5DataCollectionWrapper(env, record_dir)
            data_collector.set_randomization_info(use_randomization, seed_to_use)

        # Reset environment first
        # Temporarily set deterministic_reset to True to prevent random object placement
        original_deterministic_reset = env.deterministic_reset
        env.deterministic_reset = True
        env.reset()
        env.deterministic_reset = original_deterministic_reset
        
        # MANUAL OBJECT RANDOMIZATION
        if use_randomization and object_geoms:
            print(f"Applying MANUAL explicit color randomization to {len(object_geoms)} objects...")
            rng = np.random.RandomState(seed_to_use)
            
            for name in object_geoms:
                # Generate random color
                rgb = rng.uniform(0, 1, size=3)
                rgba = np.append(rgb, 1.0)
                
                # Set geom color
                geom_id = env.sim.model.geom_name2id(name)
                env.sim.model.geom_rgba[geom_id] = rgba
                
                # Handle material if present
                mat_id = env.sim.model.geom_matid[geom_id]
                if mat_id >= 0:
                    # Set material color
                    env.sim.model.mat_rgba[mat_id] = rgba
                    
                    # Remove texture association to force solid color
                    # Note: mat_texid is an array. For some versions it's 1D, others 2D (texture roles)
                    # We'll try to set it safely
                    try:
                        if hasattr(env.sim.model.mat_texid, 'ndim') and env.sim.model.mat_texid.ndim > 1:
                             env.sim.model.mat_texid[mat_id, :] = -1
                        else:
                             env.sim.model.mat_texid[mat_id] = -1
                    except Exception as e:
                        print(f"Warning: Could not remove texture for {name}: {e}")
                        
            # Propagate changes to renderer
            env.sim.forward()
            print("✓ Manual object randomization applied")
        
        # Start episode recording if enabled
        if data_collector:
            data_collector.start_episode()
        
        # Now load and restore ALL initial state data
        state_restored = False
        
        # Strategy 0: Try complete Mujoco state restoration first (most reliable)
        if 'initial_state' in f:
            try:
                print("✓ Attempting complete state restoration (Strategy 0)...")
                initial_state = f['initial_state'][:]
                env.sim.set_state_from_flattened(initial_state)
                env.sim.forward()
                print("✓ Successfully restored complete Mujoco state")
                state_restored = True
            except Exception as e:
                print(f"⚠ Complete state restoration failed: {e}")
                state_restored = False
        
        # Strategy 1: Try to use comprehensive joint data if available
        if not state_restored and 'initial_state_data' in f:
            try:
                print("✓ Attempting comprehensive state restoration (Strategy 1)...")
                initial_state_data = f['initial_state_data']
                
                qpos = initial_state_data['qpos'][:]
                qvel = initial_state_data['qvel'][:]
                
                # Set all joint positions and velocities
                env.sim.data.qpos[:] = qpos
                env.sim.data.qvel[:] = qvel
                
                # If body positions/rotations are available, set them too (though they derive from qpos)
                env.sim.forward()
                
                print("✓ Successfully restored all joint positions and velocities")
                state_restored = True
            except Exception as e:
                print(f"⚠ Comprehensive state restoration failed: {e}")
                state_restored = False
        
        # Strategy 2: Try explicit object joint position restoration
        if not state_restored and 'cubeA_joint_qpos' in f and 'cubeB_joint_qpos' in f:
            try:
                print("✓ Attempting explicit object position restoration (Strategy 2)...")
                cubeA_joint_qpos = f['cubeA_joint_qpos'][:]
                cubeB_joint_qpos = f['cubeB_joint_qpos'][:]
                robot_qpos = {}
                
                for i, robot in enumerate(env.robots):
                    key = f'robot{i}_joint_pos'
                    if key in f:
                        robot_qpos[key] = f[key][:]
                
                # Restore object positions
                env.sim.data.set_joint_qpos('cubeA_joint0', cubeA_joint_qpos)
                env.sim.data.set_joint_qpos('cubeB_joint0', cubeB_joint_qpos)
                
                # Restore robot positions
                for i, robot in enumerate(env.robots):
                    key = f'robot{i}_joint_pos'
                    if key in robot_qpos:
                        try:
                            robot.set_robot_joint_positions(robot_qpos[key])
                        except:
                            # Fallback: set directly in simulation
                            for j, qpos_val in enumerate(robot_qpos[key]):
                                env.sim.data.qpos[robot._ref_joint_pos_indexes[j]] = qpos_val
                
                env.sim.forward()
                print("✓ Successfully restored object and robot positions (explicit method)")
                state_restored = True
            except Exception as e:
                print(f"⚠ Explicit position restoration failed: {e}")
                state_restored = False
        
        # Strategy 3: Try using saved initial_state via set_state_from_flattened (fallback)
        if not state_restored and 'initial_state' in f:
            try:
                print("✓ Attempting standard state restoration (Strategy 3)...")
                initial_state = f['initial_state'][:]
                env.sim.set_state_from_flattened(initial_state)
                env.sim.forward()
                print("✓ Successfully restored state via set_state_from_flattened()")
                state_restored = True
            except Exception as e:
                print(f"⚠ Standard state restoration failed: {e}")
                state_restored = False
        
        if not state_restored:
            print("⚠ Could not restore initial state - using current randomized state")
            print("  Object and robot positions may not match the recorded episode")

        # Update observables after state restoration by taking a zero-action step
        if state_restored:
            zero_action = np.zeros(env.action_dim)
            obs, _, _, _ = env.step(zero_action)
            print("✓ Observables updated after state restoration")

        # Load original robot states for drift comparison
        original_robot_states = {}
        if 'robot_states' in f:
            for key in f['robot_states']:
                original_robot_states[key] = f['robot_states'][key][:]
        
        max_drift = 0.0
        
        # Playback each step
        for i, action in enumerate(actions):
            start = time.time()

            obs, reward, done, info = env.step(action)
            
            # Calculate drift
            current_drift = 0.0
            for j, robot in enumerate(env.robots):
                key = f'robot{j}_joint_pos'
                if key in original_robot_states and i < len(original_robot_states[key]):
                    current_qpos = robot._joint_positions
                    original_qpos = original_robot_states[key][i]
                    drift = np.max(np.abs(current_qpos - original_qpos))
                    current_drift = max(current_drift, drift)
            
            max_drift = max(max_drift, current_drift)
            
            # Check for task success (same as in collection)
            success = env.unwrapped._check_success()
            info['success'] = success
            
            # Record step if enabled
            if data_collector:
                data_collector.record_step(obs, action, reward, done, info)
                
            env.render()

            if i % 50 == 0:
                print(f"Playback step {i} | Reward: {reward:.3f} | Done: {done} | Drift: {current_drift:.4f} | Stability Counter: {env.unwrapped.stability_counter}")

            # Limit frame rate if necessary
            if max_fr is not None:
                elapsed = time.time() - start
                diff = 1 / max_fr - elapsed
                if diff > 0:
                    time.sleep(diff)

            if done:
                print(f"Playback terminated early at step {i} due to environment done (success)")
                break

            if dones[i]:
                print(f"Playback completed at step {i}")
                break
        
        print(f"Max drift during playback: {max_drift:.4f}")
                
        # Save recorded episode if enabled
        if data_collector:
            data_collector.save_episode()
            print(f"Saved augmented playback to {record_dir}")

    env.close()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--environment", type=str, default="MultiTableAssembly")
    parser.add_argument(
        "--robots",
        nargs="+",
        type=str,
        default="Panda",
        help="Which robot(s) to use in the env",
    )
    parser.add_argument("--directory", type=str, default="/tmp/teleop_demos/")
    parser.add_argument(
        "--device",
        type=str,
        default="keyboard",
        help="Device to use for teleoperation (keyboard, spacemouse, dualsense)",
    )
    parser.add_argument(
        "--controller",
        type=str,
        default=None,
        help="Choice of controller. Can be generic (eg. 'BASIC' or 'WHOLE_BODY_MINK_IK') or json file",
    )
    parser.add_argument(
        "--pos-sensitivity",
        type=float,
        default=1.0,
        help="How much to scale position user inputs",
    )
    parser.add_argument(
        "--rot-sensitivity",
        type=float,
        default=1.0,
        help="How much to scale rotation user inputs",
    )
    parser.add_argument(
        "--max_fr",
        default=20,
        type=int,
        help="Sleep when simulation runs faster than specified frame rate; 20 fps is real time.",
    )
    parser.add_argument(
        "--playback",
        type=str,
        default=None,
        help="Path to HDF5 file to playback instead of collecting new data",
    )
    parser.add_argument(
        "--randomize",
        action="store_true",
        help="Apply domain randomization (visuals, lighting, camera)",
    )
    parser.add_argument(
        "--randomize-seed",
        type=int,
        default=None,
        help="Seed for domain randomization",
    )
    parser.add_argument(
        "--record-playback-dir",
        type=str,
        default=None,
        help="Directory to record playback to (useful for data augmentation)",
    )

    args = parser.parse_args()

    # Get controller config
    controller_config = load_composite_controller_config(
        controller=args.controller,
        robot=args.robots[0],
    )

    # Create environment
    env = suite.make(
        args.environment,
        robots=args.robots,
        controller_configs=controller_config,
        has_renderer=True,
        has_offscreen_renderer=True,
        use_camera_obs=True,  # Enable camera observations
        use_object_obs=True,  # Enable object state observations
        camera_names=[
            "agentview",           # Main view
            "robot0_eye_in_hand",  # Wrist camera (eye-in-hand)
        ],
        camera_heights=256,
        camera_widths=256,
        ignore_done=False,
        reward_shaping=True,
        control_freq=20,
        render_camera="agentview",  # Default to wrist camera
    )

    # Wrap this environment in a visualization wrapper
    env = VisualizationWrapper(env, indicator_configs=None)

    # Setup printing options for numbers
    np.set_printoptions(formatter={"float": lambda x: "{0:0.3f}".format(x)})

    # Initialize HDF5 data collector
    data_directory = args.directory
    data_collector = HDF5DataCollectionWrapper(env, data_directory)

    if args.playback:
        # Playback mode
        print("Starting playback mode...")
        playback_trajectory(
            env, 
            args.playback, 
            args.max_fr,
            randomize=args.randomize,
            randomize_seed=args.randomize_seed,
            record_dir=args.record_playback_dir
        )
    else:
        # Teleoperation mode
        print("=" * 80)
        print("TELEOPERATION DATA COLLECTION")
        print("=" * 80)
        print(f"Environment: {args.environment}")
        print(f"Robot: {args.robots}")
        print(f"Device: {args.device}")
        print(f"Data directory: {data_directory}")
        print()
        print("Keyboard Controls:")
        print("  Ctrl+q: reset simulation")
        print("  spacebar: toggle gripper")
        print("  arrow keys: move horizontally")
        print("  . ; : move vertically")
        print("  o p: rotate (yaw)")
        print("  y h: rotate (pitch)")
        print("  e r: rotate (roll)")
        print()
        print("Starting teleoperation... Use Ctrl+q to finish episode")
        print("=" * 80)

        # initialize device
        if args.device == "keyboard":
            from robosuite.devices import Keyboard

            device = Keyboard(
                env=env,
                pos_sensitivity=args.pos_sensitivity,
                rot_sensitivity=args.rot_sensitivity,
            )
            env.viewer.add_keypress_callback(device.on_press)
        elif args.device == "spacemouse":
            from robosuite.devices import SpaceMouse

            device = SpaceMouse(
                env=env,
                pos_sensitivity=args.pos_sensitivity,
                rot_sensitivity=args.rot_sensitivity,
            )
        elif args.device == "dualsense":
            from robosuite.devices import DualSense

            device = DualSense(
                env=env,
                pos_sensitivity=args.pos_sensitivity,
                rot_sensitivity=args.rot_sensitivity,
            )
        else:
            raise Exception("Invalid device choice: choose 'keyboard', 'spacemouse', or 'dualsense'.")

        # Collect teleoperation trajectory
        collect_teleoperation_trajectory(
            env, 
            device, 
            data_collector, 
            args.max_fr,
            randomize=args.randomize,
            randomize_seed=args.randomize_seed
        )

    env.close()
    print("\nSession complete!")
