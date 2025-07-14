
# Mocap Odometry ROS 2 Node

This ROS 2 node publishes real-time odometry data of a rigid body tracked by an OptiTrack motion capture system using the `python_natnet` package. It streams both raw and filtered data to two separate topics and is designed for downstream tasks like localization, control, and visualization.

---

## Arguments

The script accepts the following arguments when launched:

- `--target-id`: Specifies the ID of the rigid body to track. The default is `47`, which corresponds to the object ID defined in the OptiTrack Motive software.
- `--server-ip`: Specifies the IP address of the OptiTrack NatNet server (i.e., the machine running Motive). The default is `130.233.123.181`.

These parameters allow the script to be reused for different tracking setups without modifying the code.

Example usage:

```bash
python3 mocap.py --target-id 47 --server-ip 130.233.123.181
```

## Network Requirements


To ensure proper operation:

- The `--server-ip` should match the IP address of the machine running the **Motive** software.
- Disable any firewalls or ensure that they allow **UDP traffic** on the required ports used by NatNet (default ports are typically `1511` for command and `1512` for data).
- Both the OptiTrack host and the ROS 2 machine should be on the **same network (LAN)**.

## ROS 2 Topics

The script publishes motion capture data to two ROS 2 topics:

---

### `/mocap/odom_raw`

- **Type**: `nav_msgs/Odometry`
- **Description**: This topic contains the raw odometry computed directly from the OptiTrack rigid body pose updates.
- **Pose Data**: Position and orientation from OptiTrack
- **Twist Data**: Linear and angular velocity computed via finite difference (no filtering)
- **Use Case**: For applications needing high-frequency, real-time data without delay or smoothing.

---

### `/mocap/odom`

- **Type**: `nav_msgs/Odometry`
- **Description**: This topic publishes a filtered version of the raw odometry using a moving average filter with a window size of 10 frames.
- **Filter Details**:
  - Linear velocity (vx, vy, vz) is averaged over the last 10 frames.
  - Angular yaw rate (z-axis) is also smoothed using a moving average.
- **Use Case**: For downstream consumers (controllers, estimators) requiring more stable, less noisy velocity data.

---

## Threading

The script uses threading to separate the NatNet client's data stream from the ROS 2 publish loop. This allows:

- Continuous reception of OptiTrack data in the background (via a separate thread)
- Independent high-rate publishing (at ~120 Hz) of ROS messages from a ROS timer callback
- Avoidance of data dropouts or race conditions between receiving and publishing

The `natnet.Client.spin()` method runs in a background thread so it can listen to OptiTrack updates independently of the ROS 2 execution loop.

---

## Dependencies

Ensure you have installed the following dependencies:

- GitHub Repository: https://github.com/mje-nz/python_natnet
