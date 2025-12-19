# RAVAGE: Robotic Autonomous Vehicles' Attack Generation Engine

**RAVAGE** is an attack injection tool that enables realistic physical attacks (e.g., GPS spoofing, gyroscope and accelerometer tampering) on robotic autonomous vehicles (RAVs) such as drones and rovers. 
---

## New additions with RAVAGE-Extended
There is now an additional flag --fault_type, -f. This flag will determine the type of fault that will be injected during the injection campaign. These fault classifications come from a 2019 paper that considers more realistic fault models. These fault types include

- `Constant Deviation Faults:` -f constant_deviation
- `Gradual Drift Faults:` -f drift_pos | drift_neg
- `Precision Damage Faults:` -f precision_damage
- `Stuck Faults:` -f stuck
- `Short Circuit Faults:` -f short_circuit

Each of these new fault types can now be included with the command to run the RAVAGE fault injector

```
@INPROCEEDINGS{8942901,
  author={Gong, Siyang and Meng, Shengwei and Wang, Benkuan and Liu, Datong},
  booktitle={2019 Prognostics and System Health Management Conference (PHM-Qingdao)}, 
  title={Hardware-In-the-Loop Simulation of UAV for Fault Injection}, 
  year={2019},
  volume={},
  number={},
  pages={1-6},
  keywords={Circuit faults;Computational modeling;Mathematical model;Actuators;Data models;Aerospace control;Prognostics and health management;UAV;HILS;Fault Injection},
  doi={10.1109/PHM-Qingdao46334.2019.8942901}}
```

## Running RAVAGE
There are two ways to run RAVAGE:

1. **Install the required tools manually** (recommended for experienced users).
2. **Use our pre-configured Ubuntu VM** for easier setup (recommended for quick deployment).

---

## Prerequisites
Ensure you have the following dependencies:
- **Python 3.X**
- **pymavlink**
- **yaml**

---

## Setup Instructions

### 1. Configure Autopilot Settings
Edit the configuration file `./config/autopilot_config.yaml` to specify your environment:

```yaml
RAVAGE_HOME: "/path/to/ravage/home/"
ARDUPILOT_HOME: "/path/to/ardupilot/home/"
PX4_HOME: "/path/to/px4/home/"
WAYPOINT_FILE: "square_copter.txt"
ROVER: "No"
COPTER: "Yes"
```

**Parameters:**
- `RAVAGE_HOME`: Path to the RAVAGE project home.
- `ARDUPILOT_HOME`: Path to the ArduPilot installation.
- `PX4_HOME`: Path to the PX4 installation.
- `WAYPOINT_FILE`: File containing mission waypoints.
- `ROVER`: Set to `Yes` for rover simulation, otherwise `No`.
- `COPTER`: Set to `Yes` for copter simulation, otherwise `No`.
---

### 2. Configure Attack Profiles
Define the attack parameters in `./config/attack_profile.yaml` to specify attack types, intensity, and duration.

---

### 3. Launch Autopilot and Simulator
Run the following command to start the autopilot and set waypoints:

```bash
python mission.py -s ArduPilot
```

**Options:**
- `-s`: Specify the autopilot system (`ArduPilot` | `PX4`)

---

### 4. Launch and Configure Sensor Attacks
Use the following command to inject attacks:

```bash
python ravage.py -s <autopilot> -a <attack_type> -i <intensity> -d <duration> -f <fault_type>
```

**Options:**
- `-s`, `--autopilot`: Autopilot software (`ArduPilot` | `PX4`)
- `-a`, `--attack_type`: Attack type (`GPS` | `GYRO` | `MAG` | `OPTICAL_FLOW` | `ACCELEROMETER`)
- `-i`, `--intensity`: Attack intensity (float value)
- `-d`, `--duration`: Attack duration (integer value)
- `-f`, `--fault_type`: Fault type (`constant_deviation` | `precision_damage` | `stuck` | `short_circuit` | `drift-pos` | `drift-neg`)
---

## Example Workflow
1. Edit the configuration files (`autopilot_config.yaml` and `attack_profile.yaml`).
2. Start the simulation and set waypoints:

```bash
python mission.py -s ArduPilot
```

3. Inject a GPS spoofing attack with intensity `3` and fault type constant deviation:

```bash
python ravage.py -s ArduPilot -a GPS -i 3 -f constant_deviation
```

This example initializes the simulation, sets mission waypoints, and performs a GPS spoofing attack during the mission.

## Citation for the original RAVAGE work, this repository is a fork of their system
```
@inproceedings{dash2025ravage,
      title={RAVAGE: Robotic Autonomous Vehicles' Attack Generation Engine}, 
      author={Dash, Pritam and Pattabiraman, Karthik},
      booktitle = {The 55th Annual IEEE/IFIP International Conference on Dependable Systems and Networks (DSN)},
      year={2025}
}
```
