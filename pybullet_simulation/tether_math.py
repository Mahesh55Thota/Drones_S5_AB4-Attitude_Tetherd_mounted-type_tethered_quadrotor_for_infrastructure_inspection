"""
Tether Geometry & Tether-Based Position Estimation
==================================================

Implementation of position estimation math from:
"Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection"
(Keigo Watanabe, Nao Moritoki, Isaku Nagai - IEEE 2017)

Paper Formulation:
------------------
Let anchor position be (x_A, y_A, z_A) on the ground plane.
Let actual quadrotor position be (x, y, z).
Displacement vector: dx = x - x_A, dy = y - y_A, dz = z - z_A.

Tether Inclination Angles:
  alpha = inclination angle to Ex-axis (X-Z plane angle relative to vertical Z-axis)
  beta  = inclination angle to Ey-axis (Y-Z plane angle relative to vertical Z-axis)
  l     = tether length = sqrt(dx^2 + dy^2 + dz^2)

Paper Equations (1), (2), (3):
  (1) x_est = z_est * tan(alpha)
  (2) y_est = z_est * tan(beta)
  (3) z_est = sqrt( (l^2 * cos(alpha)^2 * cos(beta)^2) / 
                    (cos(alpha)^2 + cos(beta)^2 - cos(alpha)^2 * cos(beta)^2) )

Position Estimation Error:
  E_est = sqrt((x - x_est)^2 + (y - y_est)^2 + (z - z_est)^2)
"""

import math
import numpy as np


def compute_tether_paper_geometry(drone_pos, anchor_pos, desired_pos=None):
    """
    Computes exact tether inclination angles alpha, beta, tether length l,
    and reconstructs estimated 3D position using Watanabe et al. (2017) Equations (1)-(3).
    """
    dx = float(drone_pos[0] - anchor_pos[0])
    dy = float(drone_pos[1] - anchor_pos[1])
    dz = float(drone_pos[2] - anchor_pos[2])

    # 1. Tether length (l)
    tether_length = math.sqrt(dx * dx + dy * dy + dz * dz)
    safe_length = max(tether_length, 1e-6)
    safe_dz = max(abs(dz), 1e-6)

    # 2. Tether inclination angles alpha (X-Z plane) and beta (Y-Z plane)
    alpha_rad = math.atan2(dx, safe_dz)
    beta_rad = math.atan2(dy, safe_dz)

    alpha_deg = math.degrees(alpha_rad)
    beta_deg = math.degrees(beta_rad)

    # 3. Paper Position Reconstruction (Equations 1, 2, 3)
    cos_a = math.cos(alpha_rad)
    cos_b = math.cos(beta_rad)

    cos_a_sq = cos_a * cos_a
    cos_b_sq = cos_b * cos_b

    denom = cos_a_sq + cos_b_sq - (cos_a_sq * cos_b_sq)
    denom = max(denom, 1e-8)

    num = (tether_length ** 2) * cos_a_sq * cos_b_sq
    z_est_rel = math.sqrt(max(0.0, num / denom))

    if dz < 0:
        z_est_rel = -z_est_rel

    x_est_rel = z_est_rel * math.tan(alpha_rad)
    y_est_rel = z_est_rel * math.tan(beta_rad)

    estimated_pos = [
        anchor_pos[0] + x_est_rel,
        anchor_pos[1] + y_est_rel,
        anchor_pos[2] + z_est_rel
    ]

    # Estimation Error
    est_error = math.sqrt(
        (drone_pos[0] - estimated_pos[0]) ** 2 +
        (drone_pos[1] - estimated_pos[1]) ** 2 +
        (drone_pos[2] - estimated_pos[2]) ** 2
    )

    # Control Tracking Error
    tracking_error = 0.0
    if desired_pos is not None:
        tracking_error = math.sqrt(
            (drone_pos[0] - desired_pos[0]) ** 2 +
            (drone_pos[1] - desired_pos[1]) ** 2 +
            (drone_pos[2] - desired_pos[2]) ** 2
        )

    # Spherical elevation and azimuth
    sin_elev = np.clip(dz / safe_length, -1.0, 1.0)
    elevation_deg = math.degrees(math.asin(sin_elev))
    azimuth_deg = math.degrees(math.atan2(dy, dx))

    return {
        "tether_length": tether_length,
        "alpha": alpha_deg,             # Inclination Ex (deg)
        "beta": beta_deg,               # Inclination Ey (deg)
        "elevation": elevation_deg,     # Elevation from ground (deg)
        "azimuth": azimuth_deg,         # Azimuth in XY plane (deg)
        "estimated_pos": estimated_pos, # (x_est, y_est, z_est) from paper formula
        "est_error": est_error,         # Estimation error (m)
        "tracking_error": tracking_error, # Control tracking error (m)
        "dx": dx,
        "dy": dy,
        "dz": dz,
    }


def vector_norm(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def normalize_vector(v):
    norm = vector_norm(v)
    if norm < 1e-8:
        return [0.0, 0.0, 0.0]
    return [v[0] / norm, v[1] / norm, v[2] / norm]


class QuadrotorAttitudePD:
    """
    Attitude PD Controller for Quadrotor from Watanabe et al. (2017) Section II.D:
    Equations (4), (5), (6):
      U2 = -K1*(phi - phi_d) - K2*phi_dot       (Roll control input)
      U3 = -K3*(theta - theta_d) - K4*theta_dot (Pitch control input)
      U4 = -K5*(psi - psi_d) - K6*psi_dot       (Yaw control input)

    Equations (7), (8):
      theta_d = -K7*(x - x_d) - K8*x_dot        (Desired pitch from position error)
      phi_d   = -K9*(y - y_d) - K10*y_dot       (Desired roll from position error)

    Principal moments of inertia from Table I:
      Ix = 0.01910 kg*m^2
      Iy = 0.01910 kg*m^2
      Iz = 0.03083 kg*m^2

    Control gains from Section IV.C:
      K1 = 0.625, K2 = 0.170  (Roll PD)
      K3 = 0.810, K4 = 0.340  (Pitch PD)
      K5 = 0.370, K6 = 0.150  (Yaw PD)
    """
    def __init__(self):
        # Gains from Watanabe et al. (2017) Section IV.C
        self.K1, self.K2 = 0.625, 0.170
        self.K3, self.K4 = 0.810, 0.340
        self.K5, self.K6 = 0.370, 0.150

        # Position-to-attitude gains (Eq 7, 8)
        self.K7, self.K8 = 0.16, 0.16
        self.K9, self.K10 = 0.16, 0.16

        # Inertia moments from Table I
        self.Ix = 0.01910
        self.Iy = 0.01910
        self.Iz = 0.03083

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.phi_dot = 0.0
        self.theta_dot = 0.0
        self.psi_dot = 0.0

    def compute_desired_angles(self, pos_error, vel):
        """Equations (7) and (8): Computes desired theta_d and phi_d."""
        theta_d = np.clip(self.K7 * pos_error[0] - self.K8 * vel[0], -0.28, 0.28)
        phi_d   = np.clip(-self.K9 * pos_error[1] - self.K10 * vel[1], -0.28, 0.28)
        return float(phi_d), float(theta_d), 0.0

    def step(self, phi_d, theta_d, psi_d=0.0, dt=1.0/30.0):
        """Equations (4), (5), (6): Updates attitude states using PD control."""
        U2 = -self.K1 * (self.phi - phi_d) - self.K2 * self.phi_dot
        U3 = -self.K3 * (self.theta - theta_d) - self.K4 * self.theta_dot
        U4 = -self.K5 * (self.psi - psi_d) - self.K6 * self.psi_dot

        # Angular accelerations from torques (Newton-Euler)
        phi_ddot = U2 / self.Ix
        theta_ddot = U3 / self.Iy
        psi_ddot = U4 / self.Iz

        # Numerical integration
        self.phi_dot += phi_ddot * dt
        self.theta_dot += theta_ddot * dt
        self.psi_dot += psi_ddot * dt

        # Realistic aerodynamic damping
        self.phi_dot *= 0.88
        self.theta_dot *= 0.88
        self.psi_dot *= 0.88

        self.phi += self.phi_dot * dt
        self.theta += self.theta_dot * dt
        self.psi += self.psi_dot * dt

        return [self.phi, self.theta, self.psi], (U2, U3, U4)

    def reset(self):
        self.phi, self.theta, self.psi = 0.0, 0.0, 0.0
        self.phi_dot, self.theta_dot, self.psi_dot = 0.0, 0.0, 0.0


class CameraStabilizerPID:
    """
    2-DOF Camera Stabilizer PID Controller from Watanabe et al. (2017) Section III.C:
    Equations (9), (10):
      U_phiC   = K11 * e_phiC   + K12 * integral(e_phiC dt)   + K13 * e_dot_phiC
      U_thetaC = K14 * e_thetaC + K15 * integral(e_thetaC dt) + K16 * e_dot_thetaC

    Equations (11), (12):
      e_phiC   = phi_Cd   - phi_C
      e_thetaC = theta_Cd - theta_C

    Gains from Section V.B (experimentally verified on robot arm):
      K11 = 170.0, K12 = 2.0, K13 = 0.1  (Roll PID)
      K14 = 170.0, K15 = 2.0, K16 = 0.1  (Pitch PID)
    """
    def __init__(self):
        # Gains from Section V.B
        self.K11, self.K12, self.K13 = 170.0, 2.0, 0.1
        self.K14, self.K15, self.K16 = 170.0, 2.0, 0.1

        self.integral_r = 0.0
        self.integral_p = 0.0
        self.prev_err_r = 0.0
        self.prev_err_p = 0.0

        # Stabilizer servo compensation angles
        self.servo_roll = 0.0
        self.servo_pitch = 0.0

        # Net stabilized camera angles (degrees)
        self.cam_roll_deg = 0.0
        self.cam_pitch_deg = 0.0

    def step(self, body_roll, body_pitch, phi_Cd=0.0, theta_Cd=0.0, dt=1.0/30.0):
        """
        Updates 2-DOF stabilizer servo positions via PID control (Eq 9-12).
        Returns stabilized camera roll and pitch angles in degrees.
        """
        # Net camera orientation = body angle + servo deflection
        cam_roll = body_roll + self.servo_roll
        cam_pitch = body_pitch + self.servo_pitch

        # Errors (Eq 11, 12)
        err_r = phi_Cd - cam_roll
        err_p = theta_Cd - cam_pitch

        # Integrals
        self.integral_r += err_r * dt
        self.integral_p += err_p * dt
        self.integral_r = np.clip(self.integral_r, -1.0, 1.0)
        self.integral_p = np.clip(self.integral_p, -1.0, 1.0)

        # Derivatives
        deriv_r = (err_r - self.prev_err_r) / max(dt, 1e-6)
        deriv_p = (err_p - self.prev_err_p) / max(dt, 1e-6)
        self.prev_err_r = err_r
        self.prev_err_p = err_p

        # PID control signals (Eq 9, 10)
        u_phi = self.K11 * err_r + self.K12 * self.integral_r + self.K13 * deriv_r
        u_theta = self.K14 * err_p + self.K15 * self.integral_p + self.K16 * deriv_p

        # Servo actuation
        self.servo_roll += u_phi * (dt * 0.15)
        self.servo_pitch += u_theta * (dt * 0.15)

        # Net camera orientation after PID stabilization
        net_roll = body_roll + self.servo_roll
        net_pitch = body_pitch + self.servo_pitch

        # Stabilized camera angle in degrees (tightly held near 0.0 per Fig. 11)
        self.cam_roll_deg = float(math.degrees(net_roll))
        self.cam_pitch_deg = float(math.degrees(net_pitch))

        return self.cam_roll_deg, self.cam_pitch_deg, (u_phi, u_theta)

    def reset(self):
        self.integral_r, self.integral_p = 0.0, 0.0
        self.prev_err_r, self.prev_err_p = 0.0, 0.0
        self.servo_roll, self.servo_pitch = 0.0, 0.0
        self.cam_roll_deg = 0.0
        self.cam_pitch_deg = 0.0

