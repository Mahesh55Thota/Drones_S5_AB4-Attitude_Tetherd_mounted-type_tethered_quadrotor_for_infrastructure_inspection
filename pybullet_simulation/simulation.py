"""
Master Tunnel Inspection Simulation Controller (Interactive Speed Control S + Clean HUD)
========================================================================================

Main simulation execution:
- PyBullet DIRECT Physics Server (Headless server mode, zero extra windows)
- Smooth 30 FPS dynamic physics simulation
- Interactive Speed Control Key 'S': Toggle between 1x, 2x, and 3x speed!
- Solid 3D MultiBody Cylinder Tether Cable (Bright red, radius 0.038m)
- High-visibility 3D Quadrotor Drone & 2-DOF Camera Gimbal
- Dynamic 3D Inspection Trajectory with side-to-side inspection sways and 20cm safety clearance
- Transparent Tunnel Environment & 3D Waypoint Markers
- Paper Tether Calculations (Watanabe et al. Equations 1-3)
- SINGLE UNIFIED WINDOW: "TETHERED QUADROTOR SIMULATION - TUNNEL INSPECTION"
"""

import time
import math
import numpy as np
import pybullet as p

from tether_math import (
    compute_tether_paper_geometry,
    normalize_vector,
    vector_norm,
    QuadrotorAttitudePD,
    CameraStabilizerPID
)
from drone_model import QuadrotorDrone
from environment import Environment
from camera_hud import DroneCameraHUD


class TetheredQuadrotorSimulation:
    def __init__(self):
        self.substeps = 8
        self.dt_substep = 1.0 / 240.0
        self.frame_dt = self.substeps * self.dt_substep  # ~0.0333 seconds per render frame (30 FPS)
        
        self.sim_time = 0.0
        self.paused = False
        self.running = True

        # Controllers from Watanabe et al. (2017)
        self.attitude_pd = QuadrotorAttitudePD()
        self.camera_pid = CameraStabilizerPID()
        self.control_torques = (0.0, 0.0, 0.0)

        # Interactive Speed Multiplier Mode (1: 1x, 2: 2x, 3: 3x)
        self.speed_mode = 1

        # Fixed Ground Tether Anchor at origin (0, 0, 0.08)
        self.anchor_position = [0.0, 0.0, 0.08]
        self.start_drone_position = [0.0, 0.0, 0.30]

        # PyBullet DIRECT Physics Setup
        if p.isConnected():
            p.disconnect()
        p.connect(p.DIRECT)
        
        p.setGravity(0.0, 0.0, -9.81)
        p.setTimeStep(self.dt_substep)

        # Initialize Environment, 3D Drone, and Single Camera HUD
        self.env = Environment(anchor_position=self.anchor_position)
        self.drone = QuadrotorDrone(start_pos=self.start_drone_position)
        self.hud = DroneCameraHUD(main_width=1280, main_height=900, pip_width=380, pip_height=285)

        # Solid 3D Tether Cylinder MultiBody ID
        self.tether_body_id = -1

        # Dynamic 3D Inspection Trajectory (Side-to-side inspection sways with 20cm clearance)
        self.waypoints = [
            {"pos": [0.0, 0.0, 1.8], "name": "1. Vertical Takeoff", "speed": 1.4, "hold": 1.2},
            {"pos": [2.0, 0.0, 1.8], "name": "2. Forward Tunnel Approach", "speed": 2.0, "hold": 1.2},
            {"pos": [3.2, -0.65, 2.0], "name": "3. Inspect Left Column Target", "speed": 1.5, "hold": 2.0},
            {"pos": [5.2, 0.0, 2.2], "name": "4. Mid-Tunnel Corridor Passage", "speed": 1.8, "hold": 1.5},
            {"pos": [7.2, 0.65, 2.2], "name": "5. Inspect Right Column Target", "speed": 1.5, "hold": 2.0},
            {"pos": [9.5, -0.55, 1.8], "name": "6. Inspect Tunnel Block 3", "speed": 1.6, "hold": 1.8},
            {"pos": [4.5, 0.0, 1.8], "name": "7. Return Flight Passage", "speed": 2.0, "hold": 1.2},
            {"pos": [0.0, 0.0, 1.8], "name": "8. Return to Ground Base", "speed": 1.5, "hold": 1.2},
            {"pos": [0.0, 0.0, 0.3], "name": "9. Precision Landing", "speed": 0.8, "hold": 2.5},
        ]
        self.current_wp_idx = 0
        self.wp_hold_start = None

        # Draw 3D Waypoint Spheres
        self.env.draw_trajectory_path(self.waypoints)

        # Kinematics state tracking
        self.current_pos = list(self.start_drone_position)
        self.current_vel = [0.0, 0.0, 0.0]
        self.current_rpy = [0.0, 0.0, 0.0]

        # Initial tether sync
        self.update_tether_visual()

    def update_tether_visual(self):
        """Updates solid 3D cylinder MultiBody tether connecting ground anchor to quadrotor base."""
        start = self.anchor_position
        end = self.current_pos

        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]

        L = math.sqrt(dx * dx + dy * dy + dz * dz)
        if L < 1e-6:
            return

        midpoint = [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (start[2] + end[2]) / 2.0]

        yaw = math.atan2(dy, dx)
        horizontal_dist = math.sqrt(dx * dx + dy * dy)
        pitch = math.atan2(horizontal_dist, max(dz, 1e-6))

        quat = p.getQuaternionFromEuler([0.0, pitch, yaw])

        if self.tether_body_id >= 0:
            p.removeBody(self.tether_body_id)

        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.038, length=L, rgbaColor=[0.95, 0.1, 0.1, 1.0])
        coll = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.038, height=L)

        self.tether_body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=coll,
            baseVisualShapeIndex=vis,
            basePosition=midpoint,
            baseOrientation=quat
        )

    def reset_simulation(self):
        """Restarts mission trajectory."""
        self.sim_time = 0.0
        self.current_wp_idx = 0
        self.wp_hold_start = None
        self.current_pos = list(self.start_drone_position)
        self.current_vel = [0.0, 0.0, 0.0]
        self.current_rpy = [0.0, 0.0, 0.0]
        self.attitude_pd.reset()
        self.camera_pid.reset()
        self.drone.set_pose(self.current_pos, self.current_rpy)
        self.update_tether_visual()
        print("\n[SIMULATION] Mission trajectory restarted.")

    def toggle_speed(self):
        """Toggles speed mode between 1x, 2x, and 3x."""
        self.speed_mode = 2 if self.speed_mode == 1 else (3 if self.speed_mode == 2 else 1)
        print(f"[SPEED] Flight speed multiplier set to {self.speed_mode}x")

    def update_drone_kinematics(self, dt):
        """Smooth waypoint trajectory control & attitude tilt calculation with speed multiplier."""
        target_wp = self.waypoints[min(self.current_wp_idx, len(self.waypoints) - 1)]
        target_pos = target_wp["pos"]
        max_speed = target_wp["speed"] * self.speed_mode
        hold_time = target_wp["hold"] / self.speed_mode

        err = [
            target_pos[0] - self.current_pos[0],
            target_pos[1] - self.current_pos[1],
            target_pos[2] - self.current_pos[2]
        ]
        dist_to_target = vector_norm(err)

        if dist_to_target < 0.10:
            self.current_vel = [0.0, 0.0, 0.0]
            if self.wp_hold_start is None:
                self.wp_hold_start = time.time()
            elif time.time() - self.wp_hold_start >= hold_time:
                if self.current_wp_idx < len(self.waypoints) - 1:
                    self.current_wp_idx += 1
                    self.wp_hold_start = None
                    print(f"[SIMULATION] Stage {self.current_wp_idx + 1}: {self.waypoints[self.current_wp_idx]['name']}")
                else:
                    print("[SIMULATION] Trajectory completed. Restarting flight mission loop...")
                    self.reset_simulation()
        else:
            dir_vec = normalize_vector(err)
            desired_speed = min(max_speed, dist_to_target * 2.5 * self.speed_mode)
            desired_vel = [dir_vec[0] * desired_speed, dir_vec[1] * desired_speed, dir_vec[2] * desired_speed]

            alpha_filter = np.clip(0.28 * self.speed_mode, 0.28, 0.65)
            for i in range(3):
                self.current_vel[i] += (desired_vel[i] - self.current_vel[i]) * alpha_filter
                self.current_pos[i] += self.current_vel[i] * dt

        # Watanabe et al. (2017) Position Controller Equations (7) & (8):
        # Computes desired pitch theta_d and desired roll phi_d from tracking error and velocity
        phi_d, theta_d, psi_d = self.attitude_pd.compute_desired_angles(err, self.current_vel)

        # Watanabe et al. (2017) Attitude PD Controller Equations (4), (5), (6):
        # Computes control torques U2, U3, U4 and closed-loop attitude integration
        rpy, torques = self.attitude_pd.step(phi_d, theta_d, psi_d, dt=dt)
        self.current_rpy = rpy
        self.control_torques = torques
        self.drone.set_pose(self.current_pos, self.current_rpy)

    def process_keyboard_inputs(self, key_opencv):
        """Processes key inputs from OpenCV window."""
        if key_opencv in [ord(' '), 32]:
            self.paused = not self.paused
            print(f"[SIMULATION] {'PAUSED' if self.paused else 'RESUMED'}")
        elif key_opencv in [ord('s'), ord('S')]:
            self.toggle_speed()
        elif key_opencv in [ord('c'), ord('C')]:
            self.hud.toggle_camera_mode()
        elif key_opencv in [ord('r'), ord('R')]:
            self.reset_simulation()
        elif key_opencv in [27, ord('q'), ord('Q')]:
            self.running = False

    def step(self):
        """Single simulation step executing substeps for smooth 30 FPS rendering."""
        if not self.paused:
            self.update_drone_kinematics(self.frame_dt)
            self.sim_time += self.frame_dt

            for _ in range(self.substeps):
                p.stepSimulation()

        # Update solid 3D cylinder tether MultiBody pose
        self.update_tether_visual()

        # Compute Tether Geometry using Watanabe Paper Formulation
        target_wp = self.waypoints[min(self.current_wp_idx, len(self.waypoints) - 1)]
        paper_geom = compute_tether_paper_geometry(
            drone_pos=self.current_pos,
            anchor_pos=self.anchor_position,
            desired_pos=target_wp["pos"]
        )

        # Compute gimbal pitch and yaw angles to point camera physically at front target object [5.2, 0.0, 2.0]
        target_dx = self.env.primary_target_pos[0] - self.current_pos[0]
        target_dy = self.env.primary_target_pos[1] - self.current_pos[1]
        target_dz = self.env.primary_target_pos[2] - self.current_pos[2]
        horiz_dist = math.sqrt(target_dx * target_dx + target_dy * target_dy)

        target_pitch = math.atan2(target_dz, max(horiz_dist, 1e-6))
        target_yaw = math.atan2(target_dy, target_dx)

        gimbal_pitch = target_pitch - self.current_rpy[1]
        gimbal_yaw = target_yaw - self.current_rpy[2]

        self.drone.set_pose(self.current_pos, self.current_rpy, gimbal_yaw=gimbal_yaw, gimbal_pitch=gimbal_pitch)

        # ── Camera Stabilizer PID Controller (Watanabe et al. 2017, Eq. 9-12) ────────
        # 2-DOF Camera Stabilizer PID (K11=170, K12=2.0, K13=0.1) dynamically compensates body tilt.
        cam_roll_deg, cam_pitch_deg, u_cam = self.camera_pid.step(
            body_roll=self.current_rpy[0],
            body_pitch=self.current_rpy[1],
            phi_Cd=0.0,
            theta_Cd=0.0,
            dt=self.frame_dt
        )
        self.drone.stabilizer_roll = self.camera_pid.servo_roll
        self.drone.stabilizer_pitch = self.camera_pid.servo_pitch

        body_roll_deg  = math.degrees(self.current_rpy[0])
        body_pitch_deg = math.degrees(self.current_rpy[1])

        telemetry = {
            "time": self.sim_time,
            "stage": target_wp["name"],
            "drone_pos": self.current_pos,
            "target_pos": target_wp["pos"],
            "tracking_target": self.env.primary_target_pos,
            "pos_error": paper_geom["tracking_error"],
            "velocity": self.current_vel,
            "tether_length": paper_geom["tether_length"],
            "alpha": paper_geom["alpha"],
            "beta": paper_geom["beta"],
            "gimbal_r": math.degrees(self.drone.stabilizer_roll),
            "gimbal_p": math.degrees(self.drone.stabilizer_pitch),
            "speed_multiplier": self.speed_mode,
            "status": "PAUSED" if self.paused else "RUNNING",
            "paused": self.paused,
            # ── Camera Stabilization Graph (Watanabe et al. 2017 - Fig. 11) ───────────
            "body_roll_deg":  body_roll_deg,   # drone body roll from PD control (Eq. 4)
            "body_pitch_deg": body_pitch_deg,  # drone body pitch from PD control (Eq. 5)
            "cam_roll_deg":   cam_roll_deg,    # stabilized camera roll from PID control (Eq. 9)
            "cam_pitch_deg":  cam_pitch_deg,   # stabilized camera pitch from PID control (Eq. 10)
        }

        # Render External 3D Perspective View
        ext_frame = self.hud.render_external_view(self.current_pos)

        # Render Drone Camera Forward View continuously locked onto Primary Target Object [5.2, 0.0, 3.2]
        cam_pos, target_pos, up_vec = self.drone.get_camera_mount_pose(target_object_pos=self.env.primary_target_pos)
        drone_cam_frame = self.hud.render_drone_camera_view(cam_pos, target_pos, up_vec)

        # Combine into Unified Single Window Frame with HUD Overlay
        composite_frame = self.hud.draw_hud_overlay(ext_frame, drone_cam_frame, telemetry)

        # Display Single OpenCV Window
        key_opencv = self.hud.show(composite_frame)
        self.process_keyboard_inputs(key_opencv)

    def run(self):
        """Main execution loop."""
        print("\n========================================================")
        print("TETHERED QUADROTOR TUNNEL INSPECTION SIMULATION")
        print("========================================================")
        print("Single Desktop Window: TETHERED QUADROTOR SIMULATION - TUNNEL INSPECTION")
        print("Controls: SPACE = Pause/Resume | S = Speed (1x/2x/3x) | C = Switch Cam | R = Restart | Q/ESC = Exit")
        print("========================================================\n")

        try:
            while self.running and p.isConnected():
                t_start = time.time()
                self.step()
                elapsed = time.time() - t_start
                sleep_time = max(0.0, self.frame_dt - elapsed)
                time.sleep(sleep_time)
        except Exception as e:
            print(f"[ERROR] Simulation loop error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean shutdown."""
        print("\n[SIMULATION] Cleaning up resources and closing windows...")
        self.hud.close()
        if p.isConnected():
            p.disconnect()
        print("[SIMULATION] Simulation closed successfully.")
