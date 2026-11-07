# Project: myPalletizer 260 Pi 4-DOF Simulation System

## Architecture
Articulated 4-DOF serial robot arm (RRRR: Base Yaw, Shoulder Pitch, Elbow Pitch, Wrist Pitch/Roll) simulation system in MATLAB R2025a and Simulink Simscape Multibody, driven by an independent first-principles kinematics engine, multi-via-point polynomial trajectory generation, and an interactive MATLAB GUI with CSV/JSON path import.

```
                           +----------------------------------------+
                           |     Interactive GUI (App / Figure)     |
                           |   (Manual Jog, FK/IK, CSV/JSON Load)   |
                           +-------------------+--------------------+
                                               |
                     +-------------------------+-------------------------+
                     |                                                   |
                     v                                                   v
     +-------------------------------+                   +-------------------------------+
     |  Trajectory Planning Engine   |                   |     Kinematics Math Engine    |
     | (Quintic Poly, Multi-Waypoint,| <---------------> |   (Pure MATLAB FK, IK, Jac,   |
     |   Pallet Grid Coordinate Gen) |                   |    DLS Inverse, Zero Corke)   |
     +---------------+---------------+                   +---------------+---------------+
                     |                                                   |
                     +-------------------------+-------------------------+
                                               |
                                               v
                           +----------------------------------------+
                           |  Simscape Multibody 3D Physical Model  |
                           | (File Solid Collada Meshes, Actuators) |
                           +----------------------------------------+
```

## Feature Inventory
Every feature identified during the Survey phase mapped to its assigned milestone:

| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| F1 | Genuine CAD/Mesh Assets | Download & sanitize official Elephant Robotics URDF & 6 Collada/STL meshes | M1 | Survey (URDF) |
| F2 | Simscape Multibody 3D Model | Complete Simscape Multibody block diagram reflecting real kinematics, links, inertia | M1 | R1, Survey |
| F3 | Joint Actuation & Sensing | Configurable joint input ports and measurement outputs for dynamic simulation | M1 | R1, R3 |
| F4 | Standard D-H Transformations | Individual homogeneous transformation matrices $T_0^1, T_1^2, T_2^3, T_3^4$ in pure MATLAB | M2 | R2, Survey |
| F5 | Forward Kinematics (FK) | Analytical FK solver mapping $[\theta_1, \theta_2, \theta_3, \theta_4] \to [x, y, z, \phi]$ | M2 | R2, Survey |
| F6 | Inverse Kinematics (IK) | Closed-form geometric IK solver with branch selection (elbow-up/down) and limit checking | M2 | R2, Survey |
| F7 | Analytical $4\times 4$ Jacobian | Exact partial-derivative Jacobian $J(\theta)$ and closed-form determinant $\det(J)$ | M2 | R2, Survey |
| F8 | Singularity Handling & DLS | Damped Least Squares pseudoinverse and manipulability monitoring | M2 | R2, Survey |
| F9 | Multi-Waypoint Pick & Place | 8-stage palletizing via-point sequence (approach, grasp, lift, transit, place, retract) | M3 | R3, Survey |
| F10 | Quintic Polynomial Trajectory | 5th-order polynomial interpolator ensuring $C^2$ position, velocity, acceleration continuity | M3 | R3, Survey |
| F11 | Pallet Grid Coordinate Generator | Automatic $M \times N \times K$ stacking matrix coordinate calculator | M3 | R3, Survey |
| F12 | Simulation Trajectory Streamer | Feed planned trajectory signals to Simscape Multibody simulation with scope logging | M3 | R3, Survey |
| F13 | GUI 3D Robot State Display | 3D kinematic plot or Simscape synchronization showing current manipulator configuration | M4 | R4, Survey |
| F14 | Manual Jog Controls | Joint sliders and Cartesian coordinate inputs with boundary checking and IK trigger | M4 | R4, Survey |
| F15 | Trajectory File Importer | Parse external CSV and JSON waypoint files, display in table, and execute | M4 | R4, Survey |
| F16 | Motion Analytics Scope | Plot joint angles, velocities, accelerations, and Cartesian tracking errors vs time | M4 | R4, Survey |
| F17 | Unit & Kinematic Verification | 20-point IK/FK round-trip test ($< 10^{-4}$ mm error), Jacobian gradient, and trajectory continuity tests | M5 / Test Track | Acceptance Criteria |
| F18 | Comprehensive E2E Test Suite | Tiers 1-4 test suite covering all features, boundaries, pairwise interactions, and workloads | M5 / Test Track | Dual Track Mandate |
| F19 | Adversarial Coverage Hardening | Tier 5 white-box stress testing, singular configurations, edge cases, and robustness | M5 | Phase 2 Mandate |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | Physical CAD Assets & 3D Simscape Multibody Model | F1, F2, F3 | None | DONE (0.00 mm FK-Simscape error, 5/5 model tests + 10/10 challenger tests + 6/6 stress tests PASS) |
| M2 | Independent Pure MATLAB Kinematics Engine | F4, F5, F6, F7, F8 | None | DONE (8 files, error 1.31e-13 mm, Gate PASS) |
| M3 | Trajectory Planning & Palletizing Motion Controller | F9, F10, F11, F12 | M2 | DONE (6 files, C^2 continuity, 15/15 unit tests + challenger stress tests PASS) |
| M4 | Interactive GUI & Trajectory File Importer | F13, F14, F15, F16 | M2, M3 | DONE (5-panel GUI, CSV/JSON importer, 31/31 unit tests + 16/16 stress tests PASS) |
| M5 | Final Milestone: E2E Test Pass (Tiers 1-4) & Adversarial Hardening (Tier 5) | F17, F18, F19 | M1, M2, M3, M4, TEST_READY.md | DONE (152/152 E2E Tests PASS, 100% pass rate, zero Peter Corke dependencies) |
| TEST | E2E Testing Track | Independent opaque-box test suite (Tiers 1-4) | None | DONE (TEST_READY.md published, 152/152 PASS) |

## Interface Contracts

### Kinematics Engine ↔ Trajectory & Simulation & GUI
- `forward_kinematics(theta)`:
  - Input: `theta` = $[1 \times 4]$ array $[ \theta_1, \theta_2, \theta_3, \theta_4 ]$ (in radians or degrees).
  - Output: `pose` = $[1 \times 4]$ array $[ x, y, z, \phi ]$ (in mm and radians/degrees) where $\phi = \theta_2 + \theta_3 + \theta_4$.
  - Output: `T_all` = $\{ T_0^1, T_0^2, T_0^3, T_0^4 \}$ homogeneous matrices $[4 \times 4]$.
- `inverse_kinematics(x, y, z, phi, config_pref, theta_current)`:
  - Input: `x, y, z` (mm), `phi` (rad), optional `config_pref` ('elbow_up' | 'elbow_down'), `theta_current` $[1 \times 4]$.
  - Output: `theta` = $[1 \times 4]$ optimal joint angles $[ \theta_1, \theta_2, \theta_3, \theta_4 ]$ within hardware limits.
  - Output: `success` = boolean flag (`true` if valid, `false` if outside workspace).
  - Output: `all_solutions` = $[K \times 4]$ matrix of all valid configurations.
- `jacobian_matrix(theta)`:
  - Input: `theta` = $[1 \times 4]$ array.
  - Output: `J` = $[4 \times 4]$ analytical Jacobian matrix.
  - Output: `det_J` = scalar determinant $r \cdot a_2 a_3 \sin\theta_3$.
  - Output: `is_singular` = boolean flag ($\det(J) < \epsilon$).
- `dls_inverse(J, lambda)`:
  - Input: `J` = $[4 \times 4]$, optional damping factor `lambda`.
  - Output: `J_inv` = $[4 \times 4]$ Damped Least Squares inverse $J^T (J J^T + \lambda^2 I)^{-1}$.

### Trajectory Planner ↔ GUI & Simscape Model
- `quintic_trajectory(s0, sf, t0, tf, t)`:
  - Input: scalar or vector start `s0`, target `sf`, start time `t0`, end time `tf`, time vector `t`.
  - Output: `s(t)` position, `ds(t)` velocity, `dds(t)` acceleration with $C^2$ continuity.
- `plan_multi_point_trajectory(waypoints, time_per_segment, dt)`:
  - Input: `waypoints` = $[N \times 4]$ matrix of $[x, y, z, \phi]$ or joint angles, duration vector $[(N-1) \times 1]$, time step `dt`.
  - Output: `traj_struct` containing `t`, `theta` $[M \times 4]$, `dtheta` $[M \times 4]$, `ddtheta` $[M \times 4]$, `xyz_actual` $[M \times 3]$.
- `pallet_grid_generator(origin, rows, cols, layers, spacing)`:
  - Input: `origin` $[x_0, y_0, z_0]$, grid dimensions, spacings $[\Delta x, \Delta y, \Delta z]$.
  - Output: `grid_points` = $[(M \times N \times K) \times 3]$ array of drop coordinates.

### Trajectory File Importer ↔ GUI
- `trajectory_parser(file_path)`:
  - Input: `file_path` (string, pointing to `.csv` or `.json`).
  - Output: `waypoints` = struct array with fields `id, x, y, z, phi, duration, action`.
  - Output: `is_valid` = boolean, `error_msg` = string (if invalid).

## Code Layout
```
d:\Lidar\myPalletizer_BTL\
├── PROJECT.md                      # Global master project plan & interface contracts
├── models\                         # Owned by Milestone 1
│   ├── mypalletizer_simscape.slx   # Simscape Multibody 3D physical model
│   ├── build_simscape_robot.m      # Automated builder and actuator config script
│   ├── urdf\                       # Cleaned URDF description files
│   └── meshes\                     # Collada DAE / STL 3D link meshes
├── kinematics\                     # Owned by Milestone 2
│   ├── dh_transform.m              # SDH homogeneous matrix generator
│   ├── forward_kinematics.m        # Analytical FK calculator
│   ├── inverse_kinematics.m        # Closed-form geometric IK solver
│   ├── jacobian_matrix.m           # 4x4 Analytical Jacobian & determinant
│   ├── dls_inverse.m               # Damped Least Squares pseudoinverse
│   └── check_limits.m              # Joint limit & workspace bounding checker
├── trajectory\                     # Owned by Milestone 3
│   ├── quintic_trajectory.m        # 5th-order polynomial trajectory interpolator
│   ├── multi_point_planner.m       # Multi-via-point pick-and-place planner
│   ├── pallet_grid_generator.m     # Pallet stacking coordinate generator
│   └── simulate_trajectory.m       # Simulink/Simscape execution & logging hook
├── gui\                            # Owned by Milestone 4
│   ├── palletizer_gui.m            # Interactive MATLAB App / Figure GUI
│   └── trajectory_parser.m         # CSV and JSON trajectory file parser
├── data\                           # Owned by Milestone 4 / Test Track
│   ├── trajectory_sample.csv       # Sample 8-step pick-and-place CSV file
│   └── trajectory_sample.json      # Sample 8-step pick-and-place JSON file
└── tests\                          # Owned by E2E Testing Track & Milestone 5
    ├── test_e2e_runner.m           # Master automated test suite runner
    ├── tier1_feature_tests.m       # Tier 1: ≥5 tests per feature
    ├── tier2_boundary_tests.m      # Tier 2: Boundary & corner cases
    ├── tier3_pairwise_tests.m      # Tier 3: Pairwise feature combinations
    └── tier4_workload_tests.m      # Tier 4: Real-world palletizing workloads
```
