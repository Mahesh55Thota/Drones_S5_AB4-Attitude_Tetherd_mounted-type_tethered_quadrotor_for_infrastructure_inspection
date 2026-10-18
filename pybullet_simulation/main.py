"""
Tethered Quadrotor Drone Simulation - Main Entry Point
======================================================

Run this script to launch the tethered quadrotor simulation:
    python main.py

Controls:
    SPACE  : Pause / Resume simulation
    R      : Restart mission trajectory
    Q/ESC  : Close simulation
"""

import sys
from simulation import TetheredQuadrotorSimulation


def main():
    sim = TetheredQuadrotorSimulation()
    sim.run()


if __name__ == "__main__":
    main()
