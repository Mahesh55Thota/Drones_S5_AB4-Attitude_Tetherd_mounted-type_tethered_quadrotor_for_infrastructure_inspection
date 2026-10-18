"""
Transparent Tunnel & Infrastructure Inspection Environment
===========================================================

Creates a 3D Tunnel Simulation Environment per IEEE Research Paper:
- Ground plane with hazard border strips
- Fixed Ground Tether Anchor (bright green metallic cylinder base with LED ring)
- Transparent Glass Tunnel Canopy with structural arch framing
- Concrete Bridge/Tunnel Pillars & Overhead Beam Inspection Targets
- Primary Inspection Target Object (Concentric Red/White Target Crosshair Plate)
- 3D Trajectory Path Visualizer (Green Waypoint Spheres & Guide Markers)
"""

import pybullet as p
import pybullet_data


class Environment:
    def __init__(self, anchor_position=(0.0, 0.0, 0.08)):
        self.anchor_position = anchor_position
        self.anchor_id = -1
        self.obstacle_ids = []
        self.waypoint_body_ids = []
        
        # Primary Target Object Position (Continuously tracked at far end of tunnel corridor after all poles)
        self.primary_target_pos = [12.5, 0.0, 2.0]
        
        self.build_environment()

    def build_environment(self):
        """Builds ground plane, anchor, transparent tunnel walls, structural arches, and obstacles."""
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Ground plane
        p.loadURDF("plane.urdf")

        # Fixed Ground Tether Anchor (Green metallic cylinder base)
        anchor_visual = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.40, length=0.18, rgbaColor=[0.1, 0.95, 0.3, 1.0]
        )
        anchor_coll = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=0.40, height=0.18
        )
        self.anchor_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=anchor_coll,
            baseVisualShapeIndex=anchor_visual,
            basePosition=self.anchor_position
        )

        # Build Transparent Tunnel & Infrastructure Obstacles
        self.create_tunnel_environment()

    def create_tunnel_environment(self):
        """Creates transparent tunnel walls, structural arch frames, and inspection pillars."""
        TUNNEL_LENGTH = 18.0
        TUNNEL_WIDTH = 7.5
        TUNNEL_HEIGHT = 5.2

        # Color Palette
        GLASS_WALL = [0.15, 0.55, 0.9, 0.14]     # Transparent blue glass wall
        FRAME_COLOR = [0.85, 0.85, 0.9, 1.0]      # Metallic arch frames
        HAZARD_YELLOW = [0.95, 0.8, 0.05, 1.0]    # Hazard stripes
        CONCRETE_PILLAR = [0.72, 0.72, 0.75, 1.0] # Concrete columns
        BEAM_COLOR = [0.95, 0.5, 0.1, 1.0]        # High-contrast orange inspection beam
        TARGET_RED = [0.95, 0.1, 0.1, 1.0]        # Red target center
        TARGET_WHITE = [0.95, 0.95, 0.95, 1.0]    # White target ring

        # 1. Floor Hazard Boundary Strips
        self._add_box(pos=[TUNNEL_LENGTH / 2.0, -TUNNEL_WIDTH / 2.0 + 0.15, 0.01], extents=[TUNNEL_LENGTH / 2.0, 0.15, 0.01], color=HAZARD_YELLOW)
        self._add_box(pos=[TUNNEL_LENGTH / 2.0, TUNNEL_WIDTH / 2.0 - 0.15, 0.01], extents=[TUNNEL_LENGTH / 2.0, 0.15, 0.01], color=HAZARD_YELLOW)

        # 2. Transparent Tunnel Canopy Walls & Ceiling
        # Left Wall
        self._add_box(pos=[TUNNEL_LENGTH / 2.0, -TUNNEL_WIDTH / 2.0, TUNNEL_HEIGHT / 2.0], extents=[TUNNEL_LENGTH / 2.0, 0.08, TUNNEL_HEIGHT / 2.0], color=GLASS_WALL)
        # Right Wall
        self._add_box(pos=[TUNNEL_LENGTH / 2.0, TUNNEL_WIDTH / 2.0, TUNNEL_HEIGHT / 2.0], extents=[TUNNEL_LENGTH / 2.0, 0.08, TUNNEL_HEIGHT / 2.0], color=GLASS_WALL)
        # Ceiling
        self._add_box(pos=[TUNNEL_LENGTH / 2.0, 0.0, TUNNEL_HEIGHT], extents=[TUNNEL_LENGTH / 2.0, TUNNEL_WIDTH / 2.0, 0.08], color=GLASS_WALL)

        # 3. Structural Arch Frames along tunnel length
        for x_frame in [0.0, 3.6, 7.2, 10.8, 14.4, 18.0]:
            # Left pillar frame
            self._add_box(pos=[x_frame, -TUNNEL_WIDTH / 2.0, TUNNEL_HEIGHT / 2.0], extents=[0.14, 0.14, TUNNEL_HEIGHT / 2.0], color=FRAME_COLOR)
            # Right pillar frame
            self._add_box(pos=[x_frame, TUNNEL_WIDTH / 2.0, TUNNEL_HEIGHT / 2.0], extents=[0.14, 0.14, TUNNEL_HEIGHT / 2.0], color=FRAME_COLOR)
            # Top beam frame
            self._add_box(pos=[x_frame, 0.0, TUNNEL_HEIGHT], extents=[0.14, TUNNEL_WIDTH / 2.0, 0.14], color=FRAME_COLOR)

        # 4. Inspection Pillars & Obstacles Inside Tunnel
        # Pillar 1 (Left column at x=3.2, y=-1.8)
        self._add_box(pos=[3.2, -1.8, 1.8], extents=[0.45, 0.45, 1.8], color=CONCRETE_PILLAR)
        self._add_box(pos=[3.2, -1.33, 2.0], extents=[0.22, 0.02, 0.22], color=TARGET_WHITE)
        self._add_box(pos=[3.2, -1.30, 2.0], extents=[0.12, 0.02, 0.12], color=TARGET_RED)

        # Overhead structural beam at x=5.2
        self._add_box(pos=[5.2, 0.0, 3.4], extents=[0.4, 2.4, 0.26], color=BEAM_COLOR)
        self._add_box(pos=[5.2, 0.0, 3.12], extents=[0.42, 2.42, 0.03], color=HAZARD_YELLOW)

        # Pillar 2 (Right column at x=7.2, y=1.8)
        self._add_box(pos=[7.2, 1.8, 2.0], extents=[0.45, 0.45, 2.0], color=CONCRETE_PILLAR)
        self._add_box(pos=[7.2, 1.33, 2.2], extents=[0.22, 0.02, 0.22], color=TARGET_WHITE)
        self._add_box(pos=[7.2, 1.30, 2.2], extents=[0.12, 0.02, 0.12], color=TARGET_RED)

        # Inspection Block 3 (at x=9.5)
        self._add_box(pos=[9.5, -1.6, 1.4], extents=[0.4, 0.4, 1.4], color=CONCRETE_PILLAR)
        self._add_box(pos=[9.5, -1.18, 1.6], extents=[0.20, 0.02, 0.20], color=TARGET_RED)

        # PRIMARY FRONT TARGET OBJECT (At x=12.5, after all poles, facing the drone corridor)
        # Target Pedestal Stand
        self._add_box(pos=[12.5, 0.0, 0.85], extents=[0.10, 0.10, 0.85], color=[0.3, 0.35, 0.4, 1.0])
        
        # Primary Target Crosshair Plate (White outer, Red inner, Golden sphere center)
        self._add_box(pos=[12.5, 0.0, 2.0], extents=[0.38, 0.38, 0.04], color=TARGET_WHITE)
        self._add_box(pos=[12.5, 0.0, 2.0], extents=[0.24, 0.24, 0.05], color=TARGET_RED)
        
        vis_core = p.createVisualShape(p.GEOM_SPHERE, radius=0.12, rgbaColor=[1.0, 0.85, 0.0, 1.0])
        coll_core = p.createCollisionShape(p.GEOM_SPHERE, radius=0.12)
        core_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=coll_core, baseVisualShapeIndex=vis_core, basePosition=[12.5, 0.0, 2.0])
        self.obstacle_ids.append(core_id)

    def _add_box(self, pos, extents, color):
        coll = p.createCollisionShape(p.GEOM_BOX, halfExtents=extents)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=extents, rgbaColor=color)
        body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=coll, baseVisualShapeIndex=vis, basePosition=pos)
        self.obstacle_ids.append(body_id)
        return body_id

    def draw_trajectory_path(self, waypoints):
        """No waypoint spheres displayed in simulation world per user request."""
        pass
