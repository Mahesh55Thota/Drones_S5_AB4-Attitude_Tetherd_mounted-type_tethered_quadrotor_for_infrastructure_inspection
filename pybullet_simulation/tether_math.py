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
