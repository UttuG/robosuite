#!/usr/bin/env python3
"""
Automated Demonstration Generation for MultiTableAssembly

This script generates demonstration trajectories programmatically without teleoperation,
using a simple algorithmic planner that sequences deterministic actions.
"""

import os
import time
import numpy as np
import robosuite as suite
from robosuite.utils.transform_utils import quat2mat, mat2quat
import h5py
from robosuite.wrappers.data_collection_wrapper import HDF5DataCollectionWrapper


class AutomatedDemonstrationGenerator:
    def __init__(self, env_name="MultiTableAssembly", robot="Panda", output_dir="/tmp/automated_demos"):
        self.env_name = env_name
        self.robot = robot
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        # Create environment
        self.env = suite.make(
            env_name,
            robots=[robot],
            has_renderer=False,
            use_camera_obs=False,
            reward_shaping=True,
            deterministic_reset=True,
        )

        # Wrap with data collector
        self.env = HDF5DataCollectionWrapper(self.env, self.output_dir)

        # Episode counter
        self.episode_count = 0

    def get_object_positions(self):
        """Extract key object positions from the environment."""
        # Get body positions
        cubeA_pos = self.env.sim.data.body_xpos[self.env.cubeA_body_id]
        cubeB_pos = self.env.sim.data.body_xpos[self.env.cubeB_body_id]

        # Table heights
        source_table_height = self.env.table_offsets[0, 2]
        dest_table_height = self.env.table_offsets[1, 2]

        return {
            'cubeA_pos': cubeA_pos,
            'cubeB_pos': cubeB_pos,
            'source_table_height': source_table_height,
            'dest_table_height': dest_table_height,
        }

    def compute_waypoints(self, positions):
        """Compute key waypoints for the task."""
        cubeA_pos = positions['cubeA_pos']
        cubeB_pos = positions['cubeB_pos']
        source_height = positions['source_table_height']
        dest_height = positions['dest_table_height']

        # Pre-grasp: above cubeA
        pre_grasp_pos = cubeA_pos.copy()
        pre_grasp_pos[2] = source_height + 0.15  # 15cm above table
        pre_grasp_quat = np.array([1, 0, 0, 0])  # Neutral orientation

        # Grasp position: at cubeA
        grasp_pos = cubeA_pos.copy()
        grasp_pos[2] = source_height + 0.05  # Just above cube
        grasp_quat = np.array([1, 0, 0, 0])

        # Lift position: higher above source
        lift_pos = cubeA_pos.copy()
        lift_pos[2] = source_height + 0.20
        lift_quat = np.array([1, 0, 0, 0])

        # Pre-place: above cubeB
        pre_place_pos = cubeB_pos.copy()
        pre_place_pos[2] = dest_height + 0.15
        pre_place_quat = np.array([1, 0, 0, 0])

        # Place position: at cubeB
        place_pos = cubeB_pos.copy()
        place_pos[2] = dest_height + 0.08  # Slightly above cubeB
        place_quat = np.array([1, 0, 0, 0])

        # Retreat position: back up after placing
        retreat_pos = cubeB_pos.copy()
        retreat_pos[2] = dest_height + 0.20
        retreat_quat = np.array([1, 0, 0, 0])

        return {
            'pre_grasp': (pre_grasp_pos, pre_grasp_quat),
            'grasp': (grasp_pos, grasp_quat),
            'lift': (lift_pos, lift_quat),
            'pre_place': (pre_place_pos, pre_place_quat),
            'place': (place_pos, place_quat),
            'retreat': (retreat_pos, retreat_quat),
        }

    def move_to_pose(self, target_pos, target_quat, steps=50, threshold=0.02):
        """Move robot to target pose using the robot's controller."""
        # Use the robot's controller to compute IK
        for _ in range(steps):
            # Get current end-effector pose
            current_pos = self.env.sim.data.site_xpos[self.env.robots[0].eef_site_id["right"]]
            current_quat = self.env.sim.data.site_xquat[self.env.robots[0].eef_site_id["right"]]

            # Compute position error
            pos_error = target_pos - current_pos
            dist = np.linalg.norm(pos_error)

            if dist < threshold:
                break

            # Use robot's controller for better IK
            # For simplicity, use proportional control in joint space
            joint_pos = self.env.robots[0]._joint_positions
            # Compute desired joint velocities (simplified)
            action_pos = pos_error * 5.0  # Higher gain
            action_ori = np.zeros(3)
            gripper_action = self.env.robots[0].gripper.current_action

            action = np.concatenate([action_pos, action_ori, [gripper_action]])
            obs, reward, done, info = self.env.step(action)

        return obs, reward, done, info

    def execute_grasp_action(self, close=True):
        """Execute gripper open/close action."""
        gripper_action = 1.0 if close else -1.0

        # Hold position, change gripper
        current_pos = self.env.sim.data.site_xpos[self.env.robots[0].eef_site_id["right"]]
        action_pos = np.zeros(6)  # No movement
        action = np.concatenate([action_pos, [gripper_action]])

        # Execute for a few steps to complete action
        for _ in range(10):
            obs, reward, done, info = self.env.step(action)

        return obs, reward, done, info

    def generate_demonstration(self):
        """Generate a single demonstration trajectory."""
        print(f"Generating demonstration {self.episode_count + 1}")

        # Reset environment
        obs = self.env.reset()

        # Start recording
        self.env.start_episode()

        # Get object positions
        positions = self.get_object_positions()
        waypoints = self.compute_waypoints(positions)

        print(f"  CubeA at: {positions['cubeA_pos']}")
        print(f"  CubeB at: {positions['cubeB_pos']}")

        # Sequence of actions
        sequence = [
            ('move_to_pre_grasp', lambda: self.move_to_pose(*waypoints['pre_grasp'])),
            ('move_to_grasp', lambda: self.move_to_pose(*waypoints['grasp'])),
            ('close_gripper', lambda: self.execute_grasp_action(close=True)),
            ('lift', lambda: self.move_to_pose(*waypoints['lift'])),
            ('move_to_pre_place', lambda: self.move_to_pose(*waypoints['pre_place'])),
            ('move_to_place', lambda: self.move_to_pose(*waypoints['place'])),
            ('open_gripper', lambda: self.execute_grasp_action(close=False)),
            ('retreat', lambda: self.move_to_pose(*waypoints['retreat'])),
        ]

        # Execute sequence
        for step_name, action_func in sequence:
            print(f"  Executing: {step_name}")
            obs, reward, done, info = action_func()

            # Check if task completed
            if info.get('success', False):
                print("  Task completed successfully!")
                break

        # Stop recording (HDF5DataCollectionWrapper saves on close)
        self.env.close()

        self.episode_count += 1

    def run_multiple_demos(self, num_demos=5):
        """Generate multiple demonstrations."""
        for i in range(num_demos):
            self.generate_demonstration()


def main():
    generator = AutomatedDemonstrationGenerator()
    generator.run_multiple_demos(num_demos=3)


if __name__ == "__main__":
    main()