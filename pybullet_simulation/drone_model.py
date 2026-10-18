"""
High-Visibility 3D Quadrotor Drone & 2-DOF Target-Tracking Stabilizer Model
===========================================================================

Creates a high-contrast, bold 3D Quadrotor Drone in PyBullet composed of MultiBodies:
1. Central Fuselage Chassis (Bright Cyan/Teal box)
2. Four Metallic Rotor Arms (+45°, +135°, +225°, +315°)
3. Four Brushless Motor Housings (Silver cylinders at arm tips)
4. Four Large Rotor Blades/Disks (Front rotors bright red, Rear rotors dark silver)
5. Front 2-DOF Camera Stabilizer Housing & Lens (Red housing per Watanabe et al. 2017)

Every part is position-updated synchronously in `set_pose(...)`.
Includes continuous target-object tracking camera lock!
"""

import math
import numpy as np
import pybullet as p


class QuadrotorDrone:
    def __init__(self, start_pos=(0.0, 0.0, 1.0), arm_length=0.48):
        self.arm_length = arm_length
        self.drone_color = [0.05, 0.65, 0.95, 1.0]  # High-contrast Cyan/Teal
        
        self.position = list(start_pos)
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 2-DOF Stabilizer Gimbal angles
        self.stabilizer_roll = 0.0
        self.stabilizer_pitch = 0.0
        self.gimbal_yaw_offset = 0.0
        
        self.part_ids = {}
        self.build_drone(start_pos)

    def _create_body(self, shape_type, half_extents=None, radius=None, height=None, color=None, pos=(0, 0, 0), quat=(0, 0, 0, 1)):
        """Helper to create a solid 3D PyBullet MultiBody object."""
        if shape_type == p.GEOM_BOX:
            coll = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
        elif shape_type == p.GEOM_CYLINDER:
            coll = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
        elif shape_type == p.GEOM_SPHERE:
            coll = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)

        body_id = p.createMultiBody(
            baseMass=0.0,  # Kinematic tracking
            baseCollisionShapeIndex=coll,
            baseVisualShapeIndex=vis,
            basePosition=pos,
            baseOrientation=quat
        )
        return body_id

    def build_drone(self, start_pos):
        """Constructs all 14 multi-body parts of the 3D Quadrotor Drone."""
        # 1. Central Fuselage Chassis
        self.part_ids["chassis"] = {
            "id": self._create_body(p.GEOM_BOX, half_extents=[0.26, 0.26, 0.07], color=self.drone_color, pos=start_pos),
            "offset": [0.0, 0.0, 0.0],
            "rot_offset": [0.0, 0.0, 0.0]
        }

        # 2. Four Rotor Arms, Motor Caps, and Propeller Disks
        arm_angles = [math.radians(45), math.radians(135), math.radians(225), math.radians(315)]
        
        for idx, angle in enumerate(arm_angles):
            dx = math.cos(angle) * self.arm_length
            dy = math.sin(angle) * self.arm_length
            
            # Arm
            arm_id = self._create_body(
                p.GEOM_CYLINDER, radius=0.03, height=self.arm_length,
                color=[0.15, 0.15, 0.20, 1.0], pos=start_pos
            )
            self.part_ids[f"arm_{idx}"] = {
                "id": arm_id,
                "offset": [dx / 2.0, dy / 2.0, 0.0],
                "rot_offset": [0.0, math.pi / 2.0, angle]
            }

            # Motor
            motor_id = self._create_body(
                p.GEOM_CYLINDER, radius=0.055, height=0.06,
                color=[0.85, 0.85, 0.90, 1.0], pos=start_pos
            )
            self.part_ids[f"motor_{idx}"] = {
                "id": motor_id,
                "offset": [dx, dy, 0.03],
                "rot_offset": [0.0, 0.0, 0.0]
            }

            # Propeller Rotor Disk (Front 2 bright red, Rear 2 dark metallic)
            prop_color = [0.95, 0.1, 0.1, 0.95] if idx in [0, 3] else [0.2, 0.2, 0.2, 0.95]
            rotor_id = self._create_body(
                p.GEOM_CYLINDER, radius=0.20, height=0.015,
                color=prop_color, pos=start_pos
            )
            self.part_ids[f"rotor_{idx}"] = {
                "id": rotor_id,
                "offset": [dx, dy, 0.07],
                "rot_offset": [0.0, 0.0, 0.0]
            }

        # 3. Front 2-DOF Camera Stabilizer Housing & Lens
        gimbal_id = self._create_body(
            p.GEOM_BOX, half_extents=[0.08, 0.06, 0.05],
            color=[0.95, 0.15, 0.15, 1.0], pos=start_pos
        )
        self.part_ids["gimbal"] = {
            "id": gimbal_id,
            "offset": [0.34, 0.0, -0.01],
            "rot_offset": [0.0, 0.0, 0.0]
        }

        # Camera lens cylinder
        lens_id = self._create_body(
            p.GEOM_CYLINDER, radius=0.03, height=0.04,
            color=[0.1, 0.1, 0.1, 1.0], pos=start_pos
        )
        self.part_ids["lens"] = {
            "id": lens_id,
            "offset": [0.42, 0.0, -0.01],
            "rot_offset": [0.0, math.pi / 2.0, 0.0]
        }

        # Initial pose sync
        self.set_pose(start_pos, [0.0, 0.0, 0.0])

    def set_pose(self, position, orientation_rpy, gimbal_yaw=0.0, gimbal_pitch=0.0):
        """Updates world position & orientation for all drone body components."""
        self.position = list(position)
        self.roll, self.pitch, self.yaw = orientation_rpy
        self.gimbal_yaw_offset = gimbal_yaw
        self.gimbal_pitch_offset = gimbal_pitch
        
        drone_quat = p.getQuaternionFromEuler(orientation_rpy)
        self.orientation = drone_quat

        # 2-DOF Camera Stabilizer PID compensation (Watanabe paper Section III)
        self.stabilizer_roll = -self.roll * 0.92
        self.stabilizer_pitch = -self.pitch * 0.92

        # Update pose of every component using p.multiplyTransforms
        for key, part in self.part_ids.items():
            offset_local = part["offset"]
            part_rpy_local = list(part["rot_offset"])
            
            # Apply camera gimbal pitch & yaw offsets to gimbal and lens
            if key in ["gimbal", "lens"]:
                part_rpy_local[1] += gimbal_pitch
                part_rpy_local[2] += gimbal_yaw
                
            part_quat_local = p.getQuaternionFromEuler(part_rpy_local)
            
            world_pos, world_quat = p.multiplyTransforms(
                position, drone_quat,
                offset_local, part_quat_local
            )
            
            p.resetBasePositionAndOrientation(part["id"], world_pos, world_quat)

    def get_forward_vector(self):
        """Returns 3D forward unit vector."""
        rot_matrix = p.getMatrixFromQuaternion(self.orientation)
        return [rot_matrix[0], rot_matrix[3], rot_matrix[6]]

    def get_camera_mount_pose(self, target_object_pos=None, forward_offset=0.42, vertical_offset=0.02):
        """
        Computes camera position and target look position attached to 2-DOF stabilizer.
        If target_object_pos is provided, continuously tracks & locks onto that specific target object!
        """
        rot_matrix = p.getMatrixFromQuaternion(self.orientation)
        drone_forward = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
        drone_up = [rot_matrix[2], rot_matrix[5], rot_matrix[8]]

        cam_x = self.position[0] + drone_forward[0] * forward_offset + drone_up[0] * vertical_offset
        cam_y = self.position[1] + drone_forward[1] * forward_offset + drone_up[1] * vertical_offset
        cam_z = self.position[2] + drone_forward[2] * forward_offset + drone_up[2] * vertical_offset

        camera_position = [cam_x, cam_y, cam_z]

        if target_object_pos is not None:
            # Continuous Object Tracking Lock directly onto target_object_pos
            target_position = list(target_object_pos)
            up_vector = [0.0, 0.0, 1.0]  # World vertical UP vector so camera horizon stays stable and object dead-center
        else:
            # Forward Stabilized View
            cam_roll = self.roll + self.stabilizer_roll
            cam_pitch = self.pitch + self.stabilizer_pitch
            cam_yaw = self.yaw + self.gimbal_yaw_offset

            cam_quat = p.getQuaternionFromEuler([cam_roll, cam_pitch, cam_yaw])
            cam_rot = p.getMatrixFromQuaternion(cam_quat)

            cam_look_dir = [cam_rot[0], cam_rot[3], cam_rot[6]]

            target_position = [
                cam_x + cam_look_dir[0] * 5.0,
                cam_y + cam_look_dir[1] * 5.0,
                cam_z + cam_look_dir[2] * 5.0
            ]
            up_vector = drone_up

        return camera_position, target_position, up_vector
