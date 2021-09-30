# User Guide & Operator Manual

This guide covers setup instructions, controls, terminal command palettes, and scenario execution for Beacon.

---

## 1. Quick Start

### 1.1 Requirements
- **Godot Engine**: Version 4.2 or later (verified on Godot 4.7.2).
- **Python**: Version 3.10 or later (for analytical tools and static scanners).
- **Git**: For version control.

### 1.2 Running the Headless Verification Suite
To execute all 21 verification suites (385 assertions covering math, spatial indexing, state estimation, kinodynamic planning, fleet orchestration, and benchmarks):

```bash
git clone https://github.com/Aneesh495/beacon.git
cd beacon
./scripts/run_tests.sh
```

A clean pass exits with status `0`:
```
========================================================================
   VERIFICATION SUMMARY REPORT   
========================================================================
  Execution Duration: 6.783 seconds
  Suites Executed:    21 (Passed: 21, Failed: 0)
  Total Assertions:   385
  Passed Assertions:  385
  Failed Assertions:  0

ALL TEST SUITES PASSED FLAWLESSLY.
========================================================================
```

### 1.3 Running the 3D Digital Twin Visualizer
To launch the interactive 3D simulation with graphics:

```bash
godot scenes/main.tscn
```

To run the full digital twin simulation headlessly (e.g. on remote servers or CI nodes):

```bash
godot --headless --path . --quit-after 100
```

---

## 2. Interactive Navigation Controls

When running in visual mode:

| Key Binding | Function |
| :--- | :--- |
| **`** (Backtick) | Toggle Developer CLI Cockpit shell |
| **1** | Switch camera to Top-Down aerial tactical overview |
| **2** | Switch camera to Third-Person vehicle follow mode |
| **3** | Switch camera to Free-Fly 6-DOF inspection |
| **4** | Switch camera to Orbital rotational inspection |
| **Tab** | Cycle primary tracked vehicle |
| **Spacebar** | Toggle simulation pause / resume |
| **W, A, S, D** | Pan camera position across road network |

---

## 3. Developer CLI Cockpit Command Manual

Press backtick (`` ` ``) to open the in-engine terminal shell.

### 3.1 System Inspection
- `status`: Displays current simulation time, tick count, real-time factor, collision metrics, and active vehicle counts.
- `help`: Displays quick reference card of available commands.
- `clear`: Clears the terminal scrollback history.

### 3.2 Fleet Operations
- `fleet list`: Prints tabular overview of all vehicles, coordinates, battery SoC, and operational states.
- `fleet dispatch <agent_id> <x> <z> [speed]`: Dispatches a vehicle to an $(X, Z)$ coordinate at a designated speed limit.
  ```text
  beacon> fleet dispatch north_0 25.0 50.0 12.0
  Dispatched north_0 to (25.0, 50.0) at 12.0 m/s
  ```
- `fleet estop [agent_id]`: Commands an immediate emergency braking stop on a target agent or the entire fleet.
- `fleet resume [agent_id]`: Clears emergency stop and restores operational readiness.

### 3.3 Benchmark Scenarios
- `scenario list`: Enumerates all pre-built benchmark scenarios.
- `scenario load <scenario_name>`: Loads, provisions, and starts a benchmark scenario:
  - `urban_grid_4way`: 12 vehicles negotiating an unsignalized intersection via AIM.
  - `warehouse_logistics`: 16 AGVs in dense rack corridors deconflicted via CBS.
  - `highway_platoon`: 6 vehicles in an adaptive cruise platoon with lead braking.
  - `sensor_degradation`: Vehicle navigating a tunnel with GPS blackout.
  - `dynamic_pedestrian_cross`: Cruising vehicle executing emergency evasive stop.

### 3.4 Chaos Engineering Fault Injection
- `fault inject <agent_id> <sensor> <fault_type> <duration_sec>`: Injects controlled hardware faults.
  - Supported sensors: `gnss`, `imu`, `lidar`, `wheel_odom`
  - Supported fault types: `blackout`, `multipath`, `bias_jump`, `high_noise`
  ```text
  beacon> fault inject north_0 gnss blackout 10.0
  Injected gnss fault 'blackout' into north_0 for 10.0s
  ```
- `fault clear`: Restores all degraded hardware components immediately.

### 3.5 Telemetry Recording & Export
- `telemetry export csv <path>`: Dumps recorded time-series flight data to CSV.
- `telemetry export json <path>`: Dumps full simulation state buffer to structured JSON.
