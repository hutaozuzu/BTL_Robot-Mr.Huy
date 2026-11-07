# Test Infrastructure & Specification Document
# 4-DOF myPalletizer 260 Pi Simulation System

**Author**: Test Writer 1 (E2E Testing Track)  
**Date**: 2026-11-07  
**Scope**: End-to-End Test Suite Architecture (Tiers 1–4)  
**Target Repository**: `d:\Lidar\myPalletizer_BTL`  

---

## 1. Testing Philosophy & Dual Track Principles

### 1.1 Dual Track Verification Mandate
The End-to-End (E2E) testing track operates under strict **Dual Track** principles:
- **Independent Authoring**: Tests are designed and authored from external specifications (`ORIGINAL_REQUEST.md`, `PROJECT.md`, `spec_report.md`), completely decoupled from the internal implementation details of individual workers.
- **Opaque-Box Requirement-Driven Testing**: Tests treat the system modules as black/opaque boxes, asserting strict compliance with interface contracts, mathematical accuracy, physical kinematic limits, and system stability.
- **Strictly Zero Peter Corke Robotics Toolbox**: All test verifications, reference mathematical oracles, and checks use native MATLAB matrix algebra and elementary mathematical operations. Any dependency on external robotics toolboxes is prohibited.
- **Progressive Testability & Graceful Execution**: When implementation modules are still in progress, tests detect module readiness, validate against contracts, and report structured status (`PASS`, `FAIL`, `PENDING_IMPLEMENTATION`) without unhandled runtime crashes.

---

## 2. Test Suite Architecture

The test suite is structured into four progressive tiers, coordinated by an automated master test runner:

```
+----------------------------------------------------------------------------------------+
|                                  test_e2e_runner.m                                     |
|                 (Master Test Orchestrator, Aggregator & Exit Code Handler)             |
+-------------------------------------------+--------------------------------------------+
                                            |
        +-----------------------------------+-----------------------------------+
        |                                   |                                   |
        v                                   v                                   v
+-----------------------+   +-----------------------+   +-----------------------+   +-----------------------+
| tier1_feature_tests.m |   | tier2_boundary_tests  |   | tier3_pairwise_tests  |   | tier4_workload_tests  |
| (F1..F19 Isolation)   |   | (Singularities/Bounds)|   | (Cross-Module Comb.)  |   | (Real-World Stacking) |
| - >=5 tests / feature |   | - r=260mm, z limits   |   | - FK + IK Round-Trip  |   | - 8-Step Pick & Place |
| - >=95 test cases     |   | - det(J)=0, r=0       |   | - IK + Trajectory     |   | - 2x2 Pallet Grid     |
|                       |   | - Joint hardware limits|  | - File Parser + Traj  |   | - 3-Layer Stacking    |
+-----------------------+   +-----------------------+   +-----------------------+   +-----------------------+
```

### 2.1 Tier Breakdown

1. **Tier 1: Feature Isolation Tests (`tier1_feature_tests.m`)**
   - Granular verification of each individual feature in the Feature Inventory (F1 through F19).
   - Minimum 5 distinct test cases per feature (19 features $\times$ 5 tests = 95 test cases).
   - Validates interface contracts, data structures, return dimensions, and core mathematical logic in isolation.

2. **Tier 2: Boundary & Corner Case Tests (`tier2_boundary_tests.m`)**
   - Rigorous evaluation of edge conditions, geometric extremes, and mathematical singularities.
   - Workspace boundaries ($R_{\max} = 260\text{ mm}$, $Z_{\max} = 358\text{ mm}$, $Z_{\min} = -15\text{ mm}$).
   - Analytical singularities ($\sin\theta_3 = 0$, $r = 0$, combined singularities).
   - Joint hardware limits (exact bounds, boundary nudges, out-of-range rejection).
   - Unreachable targets, invalid Cartesian inputs, and zero-length / reverse time intervals.

3. **Tier 3: Combinatorial Pairwise Interaction Tests (`tier3_pairwise_tests.m`)**
   - Cross-module integration tests verifying that independently built modules interoperate seamlessly.
   - Pairwise 1: Forward Kinematics $\leftrightarrow$ Inverse Kinematics round-trip consistency across all four spatial quadrants.
   - Pairwise 2: Inverse Kinematics $\leftrightarrow$ Quintic Polynomial Trajectory planner continuity.
   - Pairwise 3: Trajectory File Parser (CSV/JSON) $\leftrightarrow$ Multi-Point Trajectory Planner.
   - Pairwise 4: Pallet Grid Coordinate Generator $\leftrightarrow$ Multi-Point Trajectory Planner.
   - Pairwise 5: Analytical Jacobian $\leftrightarrow$ DLS Inverse $\leftrightarrow$ Cartesian differential tracking.
   - Pairwise 6: Trajectory Planner $\leftrightarrow$ Simulation Trajectory Streamer / Scope Logging.
   - Pairwise 7: GUI Event Handlers $\leftrightarrow$ Kinematic Engine state updates.

4. **Tier 4: Real-World Workload Tests (`tier4_workload_tests.m`)**
   - End-to-end execution of operational palletizing tasks.
   - Scenario 1: Standard 8-Step Pick-and-Place cycle (Home $\to$ Approach $\to$ Grasp $\to$ Retract $\to$ Transit $\to$ Approach $\to$ Release $\to$ Retract $\to$ Home).
   - Scenario 2: $2 \times 2$ Pallet Grid Palletizing (4 sequential item pick-and-place movements).
   - Scenario 3: 3-Layer Vertical Stacking ($1 \times 1 \times 3$ stacking with clearance altitude management).
   - Scenario 4: High-Speed Palletizing Stress Run (speed scaling $1.5\times$ and $2.0\times$ with joint velocity validation $\le 120^\circ/\text{s}$).
   - Scenario 5: Emergency Abort and Controlled Deceleration Trajectory Handling.

5. **Master Test Runner (`test_e2e_runner.m`)**
   - Discovers and executes all 4 test tiers.
   - Collects per-test status, execution duration, and assertion failures.
   - Generates summary diagnostics in tabular terminal format and exports JSON/MAT test results.
   - Exits with status code `0` on 100% pass, or code `1` if any test fails.

---

## 3. Complete Feature Coverage Mapping Table

| Feature ID | Feature Name | Tier 1 Tests | Tier 2 Tests | Tier 3 Tests | Tier 4 Tests | Primary Acceptance Criteria |
|:---|:---|:---:|:---:|:---:|:---:|:---|
| **F1** | Genuine CAD/Mesh Assets | T1_F01_01–05 | T2_B01 | T3_P06 | T4_W01 | URDF and 6 STL/DAE link meshes present, readable, valid inertia |
| **F2** | Simscape Multibody 3D Model | T1_F02_01–05 | T2_B02 | T3_P06 | T4_W01 | Simscape block diagram compiles, runs without solver NaN errors |
| **F3** | Joint Actuation & Sensing | T1_F03_01–05 | T2_B03 | T3_P06 | T4_W04 | 4 joint input ports accept motion, 4 sensor ports output angles |
| **F4** | Standard D-H Transformations | T1_F04_01–05 | T2_B04 | T3_P01 | T4_W01 | Homogeneous matrices $T_0^1..T_3^4$ orthonormal, $\det(R)=1$ |
| **F5** | Forward Kinematics (FK) | T1_F05_01–05 | T2_B05 | T3_P01 | T4_W01 | Analytical $[x, y, z, \phi]$ matches exact geometry, canonical poses |
| **F6** | Inverse Kinematics (IK) | T1_F06_01–05 | T2_B06–08 | T3_P01–02 | T4_W01–03 | Closed-form solution within limits, elbow-up/down branches |
| **F7** | Analytical $4\times 4$ Jacobian | T1_F07_01–05 | T2_B09–10 | T3_P05 | T4_W04 | Matches central difference numerical derivative $< 10^{-5}$ rel error |
| **F8** | Singularity Handling & DLS | T1_F08_01–05 | T2_B11–12 | T3_P05 | T4_W04 | Inversion bounded near $\det(J)=0$, damping $\lambda$ active |
| **F9** | Multi-Waypoint Pick & Place | T1_F09_01–05 | T2_B13 | T3_P03 | T4_W01 | 8-point cycle traversed with altitude clearances, action tags |
| **F10** | Quintic Polynomial Trajectory | T1_F10_01–05 | T2_B14–15 | T3_P02 | T4_W01–04 | $C^2$ continuity, zero boundary velocities/accelerations |
| **F11** | Pallet Grid Coordinate Generator | T1_F11_01–05 | T2_B16 | T3_P04 | T4_W02–03 | $M \times N \times K$ stacking coordinates calculated with offsets |
| **F12** | Simulation Trajectory Streamer | T1_F12_01–05 | T2_B17 | T3_P06 | T4_W01 | Trajectory data streamed to Simscape workspace variables |
| **F13** | GUI 3D Robot State Display | T1_F13_01–05 | T2_B18 | T3_P07 | T4_W01 | 3D visual axes, link plot coordinates align with FK frames |
| **F14** | Manual Jog Controls | T1_F14_01–05 | T2_B19 | T3_P07 | T4_W01 | Sliders bounded within $[-162..162, -2..90, -92..60, -180..180]^\circ$ |
| **F15** | Trajectory File Importer | T1_F15_01–05 | T2_B20 | T3_P03 | T4_W01 | Validates CSV and JSON formats, rejects corrupted inputs |
| **F16** | Motion Analytics Scope | T1_F16_01–05 | T2_B21 | T3_P06 | T4_W04 | Visualizes $\theta(t), \dot{\theta}(t), \ddot{\theta}(t)$, tracking error $< 1\text{ mm}$ |
| **F17** | Unit & Kinematic Verification | T1_F17_01–05 | T2_B05–08 | T3_P01 | T4_W01 | 20-point round trip $\|P_{\text{FK}} - P_{\text{IK}}\| < 10^{-4}\text{ mm}$ |
| **F18** | Comprehensive E2E Test Suite | T1_F18_01–05 | T2_B22 | T3_P01–07 | T4_W01–05 | Automated master test runner executes and reports status |
| **F19** | Adversarial Coverage Hardening | T1_F19_01–05 | T2_B23–25 | T3_P01–07 | T4_W04–05 | Robust against NaN, Inf, zero time intervals, extreme positions |

---

## 4. Reference Mathematical Oracles & Precision Tolerances

### 4.1 Kinematic Reference Formulas (myPalletizer 260 Pi)
- **Link Dimensions**:
  - $d_1 = 120.0\text{ mm}$ (Base height)
  - $a_2 = 140.0\text{ mm}$ (Upper arm)
  - $a_3 = 134.0\text{ mm}$ (Forearm)
  - $a_4 = 44.0\text{ mm}$ (Flange/Tool offset)
- **Forward Kinematics**:
  $$r = a_2\cos\theta_2 + a_3\cos(\theta_2 + \theta_3) + a_4\cos(\theta_2 + \theta_3 + \theta_4)$$
  $$x = r\cos\theta_1, \quad y = r\sin\theta_1$$
  $$z = d_1 + a_2\sin\theta_2 + a_3\sin(\theta_2 + \theta_3) + a_4\sin(\theta_2 + \theta_3 + \theta_4)$$
  $$\phi = \theta_2 + \theta_3 + \theta_4$$
- **Analytical Jacobian Determinant**:
  $$\det(J) = r \cdot a_2 a_3 \sin\theta_3$$
- **Quintic Polynomial**:
  $$\tau = \frac{t - t_0}{t_f - t_0}, \quad s(\tau) = s_0 + (s_f - s_0)(10\tau^3 - 15\tau^4 + 6\tau^5)$$

### 4.2 Error Tolerances & Thresholds
- **Kinematic Round-Trip Accuracy**: $\|P_{\text{original}} - P_{\text{reconstructed}}\| < 1.0 \times 10^{-4}\text{ mm}$.
- **Jacobian Derivative Relative Error**: $\frac{\|J_{\text{analytical}} - J_{\text{numerical}}\|_F}{\|J_{\text{analytical}}\|_F + \epsilon} < 1.0 \times 10^{-5}$.
- **Trajectory Smoothness ($C^2$)**: Continuous velocity and acceleration across segment junctions with maximum jump discontinuity $|\Delta \dot{\theta}| < 1.0 \times 10^{-3}\text{ rad/s}$.
- **Maximum Joint Velocity**: $|\dot{\theta}_i| \le 120^\circ/\text{s} \approx 2.0944\text{ rad/s}$.
- **Cartesian Trajectory Tracking Error**: $\|P_{\text{actual}}(t) - P_{\text{desired}}(t)\| < 1.0\text{ mm}$.

---

## 5. Execution & Verification Guide

### 5.1 Interactive MATLAB Command Window
```matlab
% Add tests and project directories to path
addpath('d:\Lidar\myPalletizer_BTL\tests');

% Run the complete E2E test suite
results = test_e2e_runner();
```

### 5.2 Headless Batch Mode (CI / Automation)
```powershell
matlab -batch "addpath('d:\Lidar\myPalletizer_BTL\tests'); exit(test_e2e_runner());"
```

### 5.3 Running Individual Tiers
```matlab
% Tier 1 only
res_t1 = tier1_feature_tests();

% Tier 2 only
res_t2 = tier2_boundary_tests();

% Tier 3 only
res_t3 = tier3_pairwise_tests();

% Tier 4 only
res_t4 = tier4_workload_tests();
```
