# 🚗 RC Car ROS 2 Data Processing and Analysis

This repository provides tools to extract, convert, and analyze data from ROS 2 `.db3` rosbags recorded using an RC car with OptiTrack motion capture. The goal is to generate interpretable CSV files from the rosbags and create visualizations for motion, control, and vehicle behavior.

---

## 🔧 `utils.py`

This file centralizes configuration and utility functions.

### Key Variables

```python
BAG_NAME = 'step_duty_1'  # Change only this to switch datasets
