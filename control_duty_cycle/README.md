#  RC Car ROS 2 Data Processing and Analysis

This repository contains utilities and scripts for extracting and analyzing data recorded from an RC car using ROS 2 bags and an OptiTrack motion capture system.

---

## Overview

The system workflow is split across three main components:

### 1. `utils.py`

- **Purpose:**  
  Contains key configuration variables and utility functions.  
- **Key variables:**  
  - `BAG_NAME`: The name of the dataset folder and rosbag file prefix. (need to only edit this) 
  - `BAG_PATH`: The path to the rosbag `.db3` file constructed using `BAG_NAME`.  
  - `PATH`: Folder path to save processed CSV files.  
  - `IS_SERVO`: Boolean flag automatically set based on whether servo command data is present in the rosbag.  

- **Important function:**  
  - `check_servo_topic_exists()`  
    Opens the specified rosbag, checks if the servo command topic `/commands/servo/position` exists, and sets `IS_SERVO` to `True` or `False`.  
    This lets downstream processing scripts adapt automatically depending on whether steering data is available.

- **How to use:**  
  Change only the `BAG_NAME` variable to switch between different rosbag datasets.  
  All other paths and settings update automatically.

---

### 2. `read_rosbags.py`

- **Purpose:**  
  Reads messages from the ROS 2 bag file and extracts selected topics into CSV files for easy analysis and visualization.

- **Topics extracted:**  
  - `/commands/motor/duty_cycle` → `duty.csv`  
  - `/commands/servo/position` → `servo.csv` (only if present)  
  - `/mocap/odom` → `odom.csv` (contains pose and velocity)  

- **Features:**  
  - Uses `utils.check_servo_topic_exists()` to determine presence of servo data and processes accordingly.  

---

### 3. Data Processing Script (e.g. `process_data.py`)

- **Purpose:**  
  Loads the CSV files generated from `read_rosbags.py` and processes the data to prepare it for visualization and analysis.

- **Key operations:**  
  - Reads `odom.csv` to extract timestamps, positions, orientations (converted from quaternion to yaw), and velocities.  
  - Reads `duty.csv` and, if present, `servo.csv`.  
  - Interpolates servo and duty cycle commands to match mocap timestamps, enabling aligned analysis.  
  - If servo data is not present (`utils.IS_SERVO` is `False`), creates a zero-valued servo command array for consistent processing.   

- **Servo command conversion:**  
  Maps raw servo input values to steering angles in radians using a calibrated linear mapping.

---

## Notes on Rosbags

- Each rosbag folder contains recordings from an experiment run of the RC car with synchronized motion capture and command data.

- **Important considerations:**  
  - In one of the bags - `step_duty_2`, the duty cycle and velocity data are not perfectly aligned. This is because the rosbag recording started while motion capture was already active but the duty cycle command recording started slightly later.  
  - For `step_duty_1` and `step_duty_2`, velocity signals may show spikes and the car's position may jitter even when the duty cycle is zero. This is due to the car being manually picked up and repositioned within the OptiTrack area to keep it inside the designated capture volume during the run.  
  - These manual movements caused the velocity to spike and position to move back and forth, which should be considered when interpreting the data.
  - The `random_1` and `longitudal_3` can be considered as test rosbags with no human intervention middle of the run.
---

## How to Use

1. Set the dataset by modifying `BAG_NAME` in `utils.py`.

2. Run `read_rosbags.py` to extract the CSV files.

3. Run the data processing and plotting script (e.g. `process_data.py`) to visualize and analyze the data.

4. The scripts automatically handle the presence or absence of servo command data.

---

## Summary

- **`utils.py`** — Configuration and detection of servo command presence.  
- **`read_rosbags.py`** — Extracts selected topics from rosbag into CSV files.  
- **Data Processing** — Loads CSVs, interpolates command signals, computes derived quantities, and generates diagnostic plots.  
- **Rosbag quirks** — Some runs contain initial misalignment and manual repositioning effects causing velocity spikes and position jitter, which is normal and expected.

---

This setup enables streamlined processing of RC car experimental data recorded via ROS 2 and OptiTrack for further control analysis and model validation.
