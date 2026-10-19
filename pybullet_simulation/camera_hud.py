"""
Clean & Unified Single Window Composite Renderer, Telemetry HUD
& Camera Stabilization Graph (Watanabe et al. IEEE 2017 - Fig. 6 style)
========================================================================

Renders a single high-definition 1280x900 window:
"TETHERED QUADROTOR SIMULATION - TUNNEL INSPECTION"

Layout:
  Top-Left overlay  : Telemetry HUD panel
  Top-Right inset   : Drone 2-DOF Camera View (PiP) with crosshair
  Bottom panel      : Camera Stabilization Graph (Roll & Pitch)
                      - Body angle (drone attitude) vs. Stabilized camera angle
                      - Exactly matches Watanabe et al. (2017) Fig. 6 style
"""

import cv2
import math
import numpy as np
import pybullet as p
from collections import deque


class DroneCameraHUD:
    WINDOW_NAME = "TETHERED QUADROTOR SIMULATION - TUNNEL INSPECTION"

    # Graph history length (samples shown at once)
    GRAPH_HISTORY = 300

    def __init__(self, main_width=1280, main_height=900, pip_width=380, pip_height=285):
        self.main_w = main_width
        self.main_h = main_height
        self.pip_w = pip_width
        self.pip_h = pip_height

        # Camera mode (0: Tunnel Chase, 1: Overview, 2: Ground Anchor Cam)
        self.cam_mode = 0

        # Projection matrices
        self.main_proj = p.computeProjectionMatrixFOV(
            fov=62.0, aspect=main_width / float(main_height),
            nearVal=0.1, farVal=100.0)
        self.drone_proj = p.computeProjectionMatrixFOV(
            fov=65.0, aspect=pip_width / float(pip_height),
            nearVal=0.05, farVal=100.0)

        # ── Stabilization Graph history buffers ──────────────────────────
        # Body angles (drone attitude): roll φ, pitch θ  [degrees]
        self.hist_body_roll  = deque([0.0] * self.GRAPH_HISTORY, maxlen=self.GRAPH_HISTORY)
        self.hist_body_pitch = deque([0.0] * self.GRAPH_HISTORY, maxlen=self.GRAPH_HISTORY)
        # Stabilized camera angles (after 2-DOF compensation) [degrees]
        self.hist_cam_roll   = deque([0.0] * self.GRAPH_HISTORY, maxlen=self.GRAPH_HISTORY)
        self.hist_cam_pitch  = deque([0.0] * self.GRAPH_HISTORY, maxlen=self.GRAPH_HISTORY)

        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WINDOW_NAME, self.main_w, self.main_h)

    # ─────────────────────────────────────────────────────────────────────
    # PyBullet renders
    # ─────────────────────────────────────────────────────────────────────

    def render_external_view(self, drone_pos):
        """Renders 3D perspective view tracking the tethered drone inside the tunnel."""
        if self.cam_mode == 0:
            cam_target = [drone_pos[0] + 1.2, drone_pos[1] * 0.5, drone_pos[2]]
            cam_eye = [
                max(-3.0, drone_pos[0] - 3.8),
                np.clip(drone_pos[1] * 0.4 - 2.2, -2.8, 2.8),
                3.2
            ]
        elif self.cam_mode == 1:
            cam_target = [4.5, 0.0, 1.8]
            cam_eye = [-3.0, 0.0, 4.2]
        else:
            cam_target = [drone_pos[0], drone_pos[1], drone_pos[2]]
            cam_eye = [-2.0, -2.5, 1.2]

        view_mat = p.computeViewMatrix(
            cameraEyePosition=cam_eye,
            cameraTargetPosition=cam_target,
            cameraUpVector=[0, 0, 1]
        )
        _, _, rgba_img, _, _ = p.getCameraImage(
            width=self.main_w, height=self.main_h,
            viewMatrix=view_mat, projectionMatrix=self.main_proj,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        img_np = np.reshape(rgba_img, (self.main_h, self.main_w, 4)).astype(np.uint8)
        return cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGR)

    def render_drone_camera_view(self, cam_pos, target_pos, up_vec):
        """Renders forward view from the drone's 2-DOF stabilized camera."""
        view_mat = p.computeViewMatrix(
            cameraEyePosition=cam_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=up_vec
        )
        _, _, rgba_img, _, _ = p.getCameraImage(
            width=self.pip_w, height=self.pip_h,
            viewMatrix=view_mat, projectionMatrix=self.drone_proj,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        img_np = np.reshape(rgba_img, (self.pip_h, self.pip_w, 4)).astype(np.uint8)
        return cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGR)

    # ─────────────────────────────────────────────────────────────────────
    # Camera Mode toggle
    # ─────────────────────────────────────────────────────────────────────

    def toggle_camera_mode(self):
        self.cam_mode = (self.cam_mode + 1) % 3
        mode_names = ["Tunnel Chase Cam", "Entrance Overview", "Ground Anchor Cam"]
        print(f"[CAMERA] Switched to View Mode {self.cam_mode + 1}: {mode_names[self.cam_mode]}")

    # ─────────────────────────────────────────────────────────────────────
    # Stabilization graph (pure OpenCV — no matplotlib)
    # ─────────────────────────────────────────────────────────────────────

    def _draw_stabilization_graph(self, frame, telemetry):
        """
        Draws the Camera Stabilization Graph panel at the bottom of the frame.
        Shows Roll and Pitch: body angle (dashed) vs stabilized camera angle (solid flat 0°).
        Style matches Watanabe et al. (2017) IEEE paper Fig. 6.
        Freezes when simulation is paused.
        """
        is_paused = telemetry.get("paused", False)

        # Only push new data when simulation is NOT paused
        if not is_paused:
            body_roll_deg  = telemetry.get("body_roll_deg",  0.0)
            body_pitch_deg = telemetry.get("body_pitch_deg", 0.0)
            cam_roll_deg   = telemetry.get("cam_roll_deg",   0.0)
            cam_pitch_deg  = telemetry.get("cam_pitch_deg",  0.0)

            self.hist_body_roll.append(body_roll_deg)
            self.hist_body_pitch.append(body_pitch_deg)
            self.hist_cam_roll.append(cam_roll_deg)
            self.hist_cam_pitch.append(cam_pitch_deg)

        # ── Graph panel geometry ─────────────────────────────────────────
        graph_panel_h = 200          # total bottom panel height
        graph_panel_y = self.main_h - graph_panel_h  # top y of panel
        panel_bg = (12, 16, 24)

        # Dark background
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, graph_panel_y), (self.main_w, self.main_h), panel_bg, -1)
        cv2.addWeighted(overlay, 0.88, frame, 0.12, 0, frame)
        # Top border line
        cv2.line(frame, (0, graph_panel_y), (self.main_w, graph_panel_y), (0, 220, 140), 1)

        # Panel title + paused badge
        cv2.putText(frame, "CAMERA STABILIZATION GRAPH  (Watanabe et al. 2017 - Eq. 2-DOF)",
                    (14, graph_panel_y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)
        if is_paused:
            cv2.putText(frame, "[ PAUSED — Graph Frozen ]",
                        (self.main_w - 260, graph_panel_y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 165, 255), 1, cv2.LINE_AA)

        # Split into two sub-graphs: Roll (left half) | Pitch (right half)
        half_w = self.main_w // 2
        margin_x, margin_y_top, margin_y_bot = 60, 34, 28
        graph_h = graph_panel_h - margin_y_top - margin_y_bot  # usable height inside sub-graph
        y_scale_deg = 20.0  # ±20° displayed range

        for sub_idx, (label, hist_body, hist_cam, body_color, cam_color) in enumerate([
            # Roll  — Body: bright orange  | Camera: bright lime green
            ("Roll  φ (deg)",  self.hist_body_roll,  self.hist_cam_roll,
             (0, 140, 255), (0, 255, 80)),
            # Pitch — Body: magenta/pink   | Camera: bright yellow
            ("Pitch θ (deg)", self.hist_body_pitch, self.hist_cam_pitch,
             (200, 60, 255), (0, 230, 255)),
        ]):
            # Sub-graph bounding box
            sg_x0 = sub_idx * half_w + margin_x
            sg_x1 = (sub_idx + 1) * half_w - 20
            sg_y0 = graph_panel_y + margin_y_top
            sg_y1 = sg_y0 + graph_h
            sg_w  = sg_x1 - sg_x0

            # Background & border
            cv2.rectangle(frame, (sg_x0, sg_y0), (sg_x1, sg_y1), (20, 26, 36), -1)
            cv2.rectangle(frame, (sg_x0, sg_y0), (sg_x1, sg_y1), (60, 80, 100), 1)

            # Sub-title
            cv2.putText(frame, label, (sg_x0, sg_y0 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)

            # Helper: degree → pixel y (zero-line is at centre)
            def deg2py(val):
                clamped = max(-y_scale_deg, min(y_scale_deg, val))
                ratio = clamped / y_scale_deg          # -1..+1
                return int((sg_y0 + sg_y1) / 2 - ratio * (graph_h / 2))

            # Zero-centre line
            cy = (sg_y0 + sg_y1) // 2
            cv2.line(frame, (sg_x0, cy), (sg_x1, cy), (60, 80, 100), 1)

            # Grid lines at ±5°, ±10°, ±15°, ±20°
            for tick_deg in [5, 10, 15, 20]:
                for sign in [1, -1]:
                    gy = deg2py(sign * tick_deg)
                    cv2.line(frame, (sg_x0, gy), (sg_x1, gy), (30, 45, 55), 1)
                    cv2.putText(frame, f"{sign*tick_deg:+d}°",
                                (sg_x0 - margin_x + 2, gy + 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.30, (130, 150, 160), 1, cv2.LINE_AA)

            # Draw both traces (body = dashed look via sampling every other pt, cam = solid)
            n = len(hist_body)
            x_step = sg_w / max(n - 1, 1)

            def sample_x(i):
                return int(sg_x0 + i * x_step)

            # Body angle trace (drone attitude) — dashed cyan/blue
            pts_body = [(sample_x(i), deg2py(v)) for i, v in enumerate(hist_body)]
            for i in range(1, len(pts_body)):
                if i % 4 != 0:   # skip every 4th segment → dashed effect
                    cv2.line(frame, pts_body[i-1], pts_body[i], body_color, 1, cv2.LINE_AA)

            # Stabilized camera angle trace — solid bright green/yellow
            pts_cam = [(sample_x(i), deg2py(v)) for i, v in enumerate(hist_cam)]
            for i in range(1, len(pts_cam)):
                cv2.line(frame, pts_cam[i-1], pts_cam[i], cam_color, 2, cv2.LINE_AA)

            # Current value dots
            cv2.circle(frame, pts_body[-1], 4, body_color, -1)
            cv2.circle(frame, pts_cam[-1],  4, cam_color,  -1)

            # X-Axis Label (Time)
            cv2.putText(frame, "Time (last 10s) ->", (sg_x1 - 110, sg_y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1, cv2.LINE_AA)

            # Legend (bottom left of sub-graph)
            leg_x = sg_x0 + 10
            leg_y = sg_y1 - 10
            cv2.line(frame,  (leg_x, leg_y), (leg_x + 22, leg_y), body_color, 1)
            cv2.putText(frame, "Body angle",
                        (leg_x + 26, leg_y + 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.34, body_color, 1, cv2.LINE_AA)
            leg_y2 = leg_y - 14
            cv2.line(frame,  (leg_x, leg_y2), (leg_x + 22, leg_y2), cam_color, 2)
            cv2.putText(frame, "Stabilized camera angle",
                        (leg_x + 26, leg_y2 + 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.32, cam_color, 1, cv2.LINE_AA)

        return frame

    # ─────────────────────────────────────────────────────────────────────
    # Main composite render
    # ─────────────────────────────────────────────────────────────────────

    def draw_hud_overlay(self, main_frame, drone_cam_frame, telemetry):
        """
        Combines external tunnel view, embedded drone camera inset,
        clean telemetry HUD overlay, and stabilization graph into ONE window.
        """
        h, w, _ = main_frame.shape
        margin = 18

        # ── 1. PiP Drone Camera (top-right) ─────────────────────────────
        pip_x1 = w - self.pip_w - margin
        pip_y1 = margin + 35
        pip_x2 = pip_x1 + self.pip_w
        pip_y2 = pip_y1 + self.pip_h

        main_frame[pip_y1:pip_y2, pip_x1:pip_x2] = drone_cam_frame

        cv2.rectangle(main_frame, (pip_x1 - 2, pip_y1 - 26),
                      (pip_x2 + 2, pip_y2 + 2), (0, 220, 140), 2)
        cv2.rectangle(main_frame, (pip_x1 - 2, pip_y1 - 26),
                      (pip_x2 + 2, pip_y1 - 2), (20, 24, 32), -1)
        cv2.putText(main_frame, "DRONE CAMERA VIEW (2-DOF STABILIZER)",
                    (pip_x1 + 8, pip_y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 180), 1, cv2.LINE_AA)

        # Crosshair
        pip_cx = pip_x1 + self.pip_w // 2
        pip_cy = pip_y1 + self.pip_h // 2
        cv2.line(main_frame, (pip_cx - 16, pip_cy), (pip_cx + 16, pip_cy), (0, 255, 180), 1)
        cv2.line(main_frame, (pip_cx, pip_cy - 16), (pip_cx, pip_cy + 16), (0, 255, 180), 1)
        cv2.circle(main_frame, (pip_cx, pip_cy), 5, (0, 255, 180), 1)

        # ── 2. Telemetry HUD panel (top-left) ───────────────────────────
        panel_w, panel_h = 450, 305
        overlay = main_frame.copy()
        cv2.rectangle(overlay, (margin, margin),
                      (margin + panel_w, margin + panel_h), (12, 16, 24), -1)
        cv2.addWeighted(overlay, 0.75, main_frame, 0.25, 0, main_frame)
        cv2.rectangle(main_frame, (margin, margin),
                      (margin + panel_w, margin + panel_h), (0, 220, 140), 1)

        cv2.putText(main_frame, "TETHERED QUADROTOR SIMULATION",
                    (margin + 14, margin + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(main_frame,
                 (margin + 14, margin + 34),
                 (margin + panel_w - 14, margin + 34), (0, 220, 140), 1)

        drone_pos  = telemetry.get("drone_pos",       [0, 0, 0])
        tgt_pos    = telemetry.get("target_pos",      [0, 0, 0])
        trk_tgt    = telemetry.get("tracking_target", [12.5, 0.0, 2.0])
        vel        = telemetry.get("velocity",         [0, 0, 0])
        speed      = np.linalg.norm(vel)
        status_str = telemetry.get("status",           "RUNNING")
        status_color = (0, 255, 0) if status_str == "RUNNING" else (0, 165, 255)
        stage_name = telemetry.get("stage",            "Inspection")
        speed_mult = telemetry.get("speed_multiplier", 1)
        mode_names = ["Tunnel Chase Cam", "Entrance Overview", "Ground Anchor Cam"]
        cam_str    = mode_names[self.cam_mode]

        lines = [
            (f"Stage: {stage_name}", (255, 220, 100)),
            (f"Status: {status_str}  |  Sim Time: {telemetry.get('time', 0.0):.2f} s", status_color),
            (f"Tracking Target:  [{trk_tgt[0]:.2f}, {trk_tgt[1]:.2f}, {trk_tgt[2]:.2f}] m", (0, 255, 180)),
            (f"Drone Pos (x,y,z):   ({drone_pos[0]:+.2f}, {drone_pos[1]:+.2f}, {drone_pos[2]:+.2f}) m", (200, 240, 255)),
            (f"Waypoint Pos:        ({tgt_pos[0]:+.2f}, {tgt_pos[1]:+.2f}, {tgt_pos[2]:+.2f}) m", (200, 240, 255)),
            (f"Tether Length (l):   {telemetry.get('tether_length', 0.0):.3f} m", (120, 255, 120)),
            (f"Inclination Alpha:   {telemetry.get('alpha', 0.0):+.2f} deg (Ex-axis)", (120, 255, 255)),
            (f"Inclination Beta:    {telemetry.get('beta',  0.0):+.2f} deg (Ey-axis)", (120, 255, 255)),
            (f"Tracking Error:      {telemetry.get('pos_error', 0.0):.3f} m", (100, 220, 255)),
            (f"Speed: {speed:.2f} m/s [{speed_mult}x]  |  View: {cam_str}", (240, 240, 240)),
        ]

        y_off = margin + 58
        for text, color in lines:
            cv2.putText(main_frame, text, (margin + 14, y_off),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, color, 1, cv2.LINE_AA)
            y_off += 24

        # ── 3. Camera Stabilization Graph (bottom panel) ─────────────────
        main_frame = self._draw_stabilization_graph(main_frame, telemetry)

        # ── 4. Controls hint bar (above the graph panel) ─────────────────
        ctrl_y = self.main_h - 200 - 22
        cv2.putText(main_frame,
                    "CONTROLS:  SPACE=Pause/Resume  |  S=Speed(1x/2x/3x)  |  C=Switch Cam  |  R=Restart  |  Q/ESC=Exit",
                    (14, ctrl_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)

        return main_frame

    # ─────────────────────────────────────────────────────────────────────

    def show(self, frame):
        cv2.imshow(self.WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        return key

    def close(self):
        cv2.destroyAllWindows()
