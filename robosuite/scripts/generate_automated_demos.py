
import os
import time
import h5py
import numpy as np
import robosuite as suite
from robosuite.controllers import load_part_controller_config
from robosuite.controllers.composite.composite_controller_factory import refactor_composite_controller_config
from robosuite.utils.transform_utils import convert_quat, quat2mat, mat2quat, axisangle2quat, quat_multiply
from robosuite.utils.control_utils import orientation_error

class HDF5DataCollectionWrapper:
    """
    HDF5-based data collection wrapper for recording demonstrations.
    """

    def __init__(self, env, directory, collect_freq=1, flush_freq=100):
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
            if self.current_episode_data['robot_states']:
                robot_group = f.create_group('robot_states')
                for key in self.current_episode_data['robot_states'][0].keys():
                    data = [state[key] for state in self.current_episode_data['robot_states']]
                    robot_group.create_dataset(key, data=np.array(data))

            # Save gripper states
            if self.current_episode_data['gripper_states']:
                gripper_group = f.create_group('gripper_states')
                for key in self.current_episode_data['gripper_states'][0].keys():
                    data = [state[key] for state in self.current_episode_data['gripper_states']]
                    gripper_group.create_dataset(key, data=np.array(data))

            # Save observables
            if self.current_episode_data['observables']:
                obs_group = f.create_group('observables')
                for key in self.current_episode_data['observables'][0].keys():
                    data = [obs[key] for obs in self.current_episode_data['observables']]
                    obs_group.create_dataset(key, data=np.array(data))

    def step(self, action):
        """
        Extends vanilla step() function call to accommodate data collection
        """
        obs, reward, done, info = self.env.step(action)
        self.record_step(obs, action, reward, done, info)
        return obs, reward, done, info
        
    def reset(self):
        """
        Extends vanilla reset() function call to accommodate data collection
        """
        obs = self.env.reset()
        return obs
        
    def close(self):
        """
        Override close method in order to flush left over data
        """
        self.save_episode()
        self.env.close()
        
    def __getattr__(self, attr):
        """
        Delegate attribute access to the underlying environment.
        """
        return getattr(self.env, attr)


class AutomatedDemoGenerator:
    def __init__(self, env_name="MultiTableAssembly", robot="Panda", output_dir="/tmp/automated_demos"):
        self.env_name = env_name
        self.robot = robot
        self.output_dir = output_dir
        
        # Load OSC_POSE controller configuration
        controller_config = load_part_controller_config(default_controller="OSC_POSE")
        controller_config = refactor_composite_controller_config(controller_config, robot, ["right"])
        
        # Create environment
        self.env = suite.make(
            env_name,
            robots=[robot],
            controller_configs=controller_config,
            has_renderer=False,           # Headless for speed
            use_camera_obs=True,          # Needed for data collection (images)
            use_object_obs=True,
            horizon=2000,
            reward_shaping=True,
            control_freq=20,
        )
        
        # Wrap with Data Collection Wrapper
        self.env = HDF5DataCollectionWrapper(self.env, self.output_dir)
        
        self.episode_count = 0

    def get_target_poses(self):
        """
        Calculate the critical waypoints based on the current environment state.
        """
        sim = self.env.sim
        
        # Get object IDs
        cubeA_id = self.env.cubeA_body_id
        cubeB_id = self.env.cubeB_body_id
        
        # Get current positions
        cubeA_pos = np.array(sim.data.body_xpos[cubeA_id])
        cubeB_pos = np.array(sim.data.body_xpos[cubeB_id])
        
        # Calculate grasp orientation aligned with cube
        cubeA_quat = sim.data.body_xquat[cubeA_id] # [w, x, y, z]
        cubeA_quat_xyzw = convert_quat(cubeA_quat, to="xyzw")
        
        # Rotate 180 around X to flip Z down (align gripper with cube)
        # This assumes cube Z is up.
        rot_quat = axisangle2quat(np.array([np.pi, 0, 0]))
        grasp_quat = quat_multiply(cubeA_quat_xyzw, rot_quat)
        
        waypoints = {}
        
        # 1. Pre-Grasp: Above Cube A
        waypoints['pre_grasp_pos'] = cubeA_pos + np.array([0, 0, 0.10])
        waypoints['pre_grasp_quat'] = grasp_quat
        
        # 2. Grasp: At Cube A
        # Go to center (should fit if aligned and open)
        waypoints['grasp_pos'] = cubeA_pos + np.array([0, 0, 0.00])
        waypoints['grasp_quat'] = grasp_quat
        
        # 3. Lift: Above Cube A
        waypoints['lift_pos'] = cubeA_pos + np.array([0, 0, 0.15])
        waypoints['lift_quat'] = grasp_quat
        
        # 4. Pre-Place: Above Cube B
        waypoints['pre_place_pos'] = cubeB_pos + np.array([0, 0, 0.15])
        waypoints['pre_place_quat'] = grasp_quat
        
        # 5. Place: At Cube B (stacking)
        # Target height: Cube B half-height (0.04) + Cube A half-height (~0.0375) = 0.0775
        # We add a small buffer to avoid collision before release
        waypoints['place_pos'] = cubeB_pos + np.array([0, 0, 0.08]) 
        waypoints['place_quat'] = grasp_quat
        
        return waypoints

    def move_to_pose(self, target_pos, target_quat, gripper_closed=False, tolerance=0.01, max_steps=500):
        """
        Move the EE to the target pose using the OSC controller.
        """
        # Logic flipped: 1.0 is closed, -1.0 is open for this controller/robot config apparently
        gripper_action = -1.0 if not gripper_closed else 1.0
        
        for i in range(max_steps):
            # Get current EE state
            current_pos = self.env.sim.data.site_xpos[self.env.robots[0].eef_site_id["right"]]
            current_mat = self.env.sim.data.site_xmat[self.env.robots[0].eef_site_id["right"]].reshape(3, 3)
            current_quat = mat2quat(current_mat)
            
            # Calculate error
            pos_error = target_pos - current_pos
            ori_error = orientation_error(quat2mat(target_quat), current_mat)
            
            # Simple P-controller
            kp_pos = 5.0
            kp_ori = 2.0
            
            action_pos = np.clip(pos_error * kp_pos, -1, 1)
            action_ori = np.clip(ori_error * kp_ori, -1, 1)
            
            # Construct action: [x, y, z, ax, ay, az, gripper]
            action = np.concatenate([action_pos, action_ori, [gripper_action]])
            
            # Step environment
            obs, reward, done, info = self.env.step(action)
            
            # Check convergence
            if np.linalg.norm(pos_error) < tolerance and np.linalg.norm(ori_error) < 0.1:
                return True, info
                
            if done:
                return False, info
                
        print(f"Warning: move_to_pose timed out. Pos Error: {np.linalg.norm(pos_error):.4f}, Ori Error: {np.linalg.norm(ori_error):.4f}")
        return False, info

    def execute_gripper(self, closed=True, steps=50):
        """
        Execute gripper action (open/close) in place.
        """
        # Logic flipped: 1.0 is closed, -1.0 is open
        gripper_action = -1.0 if not closed else 1.0
        action = np.zeros(7)
        action[-1] = gripper_action
        
        for _ in range(steps):
            obs, reward, done, info = self.env.step(action)
            if done:
                break

    def generate_demo(self):
        print(f"Generating Demo {self.episode_count}...")
        obs = self.env.reset()
        
        # Start recording
        self.env.start_episode()
        
        waypoints = self.get_target_poses()
        
        # 1. Move to Pre-Grasp
        print("  Moving to Pre-Grasp...")
        self.move_to_pose(waypoints['pre_grasp_pos'], waypoints['pre_grasp_quat'], gripper_closed=False)
        
        # 2. Move to Grasp
        print("  Moving to Grasp...")
        self.move_to_pose(waypoints['grasp_pos'], waypoints['grasp_quat'], gripper_closed=False)
        
        # 3. Close Gripper
        print("  Closing Gripper...")
        self.execute_gripper(closed=True)
        
        # 4. Lift
        print("  Lifting...")
        self.move_to_pose(waypoints['lift_pos'], waypoints['lift_quat'], gripper_closed=True)
        
        # 5. Move to Pre-Place
        print("  Moving to Pre-Place...")
        self.move_to_pose(waypoints['pre_place_pos'], waypoints['pre_place_quat'], gripper_closed=True)
        
        # 6. Move to Place
        print("  Placing...")
        self.move_to_pose(waypoints['place_pos'], waypoints['place_quat'], gripper_closed=True)
        
        # 7. Open Gripper
        print("  Opening Gripper...")
        self.execute_gripper(closed=False)
        
        # Wait for object to settle
        print("  Waiting for settle...")
        for _ in range(20):
            self.env.step(np.zeros(7))
        
        # 8. Retreat
        print("  Retreating...")
        retreat_pos = waypoints['place_pos'] + np.array([0, 0, 0.15])
        self.move_to_pose(retreat_pos, waypoints['place_quat'], gripper_closed=False)
        
        # Check success
        r_reach, r_lift, r_stack = self.env.staged_rewards()
        print(f"  Rewards - Reach: {r_reach:.2f}, Lift: {r_lift:.2f}, Stack: {r_stack:.2f}")
        
        # DEBUG
        cubeA_pos = self.env.sim.data.body_xpos[self.env.cubeA_body_id]
        cubeB_pos = self.env.sim.data.body_xpos[self.env.cubeB_body_id]
        
        target_table_height = self.env.table_offsets[1, 2]
        cubeA_height = cubeA_pos[2]
        cubeA_at_stack_height = cubeA_height > target_table_height + 0.04
        
        grasping_cubeA = self.env._check_grasp(gripper=self.env.robots[0].gripper, object_geoms=self.env.cubeA)
        
        cubeA_bottom = cubeA_pos[2] - 0.0375
        cubeB_top = cubeB_pos[2] + 0.04
        vertical_contact = abs(cubeA_bottom - cubeB_top) < 0.01
        
        print(f"DEBUG: Height OK: {cubeA_at_stack_height} ({cubeA_height:.4f} > {target_table_height + 0.04:.4f})")
        print(f"DEBUG: Not Grasping: {not grasping_cubeA}")
        print(f"DEBUG: Vert Contact: {vertical_contact} (Diff: {abs(cubeA_bottom - cubeB_top):.4f})")
        print(f"DEBUG: Stability Counter: {self.env.stability_counter}/{self.env.stability_threshold}")
        
        success = self.env._check_success()
        print(f"  Episode Success: {success}")
        
        # Flush/Save
        self.env.close()
        self.episode_count += 1

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_demos", type=int, default=5, help="Number of demos to generate")
    args = parser.parse_args()
    
    generator = AutomatedDemoGenerator()
    for i in range(args.num_demos):
        generator.generate_demo()
