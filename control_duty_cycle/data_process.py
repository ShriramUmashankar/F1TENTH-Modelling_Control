import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import utils

utils.check_servo_topic_exists()


def quaternion_to_yaw(x, y, z, w):
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y**2 + z**2)
    return np.arctan2(t3, t4)


def read_odom():
    data = np.loadtxt(utils.PATH + 'odom.csv', delimiter=',', skiprows=1)
    t = data[:, 0] - data[0, 0]
    x = data[:, 1] - data[0, 1]
    y = data[:, 2] - data[0, 2]
    yaw = np.unwrap(quaternion_to_yaw(data[:, 4], data[:, 5], data[:, 6], data[:, 7]))
    vx = data[:, 8]
    vy = data[:, 9]
    r = data[:, 10]
    return t, x, y, yaw, vx, vy, r


def read_duty():
    data = np.loadtxt(utils.PATH + 'duty.csv', delimiter=',', skiprows=1)
    t = data[:, 0] - data[0, 0]
    val = data[:, 1]
    return t, val


def read_servo():
    data = np.loadtxt(utils.PATH + 'servo.csv', delimiter=',', skiprows=1)
    t = data[:, 0] - data[0, 0]
    val = data[:, 1]
    return t, val
        

def servo_to_angle_radians(servo_value):
    servo_min = 0.15
    servo_max = 0.85
    angle_min_deg = 19.5
    angle_max_deg = -19.5

    servo_value = np.asarray(servo_value)
    servo_value = np.clip(servo_value, servo_min, servo_max)

    proportion = (servo_value - servo_min) / (servo_max - servo_min)
    angle_deg = angle_min_deg + proportion * (angle_max_deg - angle_min_deg)
    return np.radians(angle_deg)


# === Load data ===
odom_t, odom_x, odom_y, odom_yaw, odom_vx, odom_vy, odom_r = read_odom()
odom_speed = np.sqrt(odom_vx**2 + odom_vy**2)

# Interpolate duty cycle to mocap time
duty_t, duty_val = read_duty()
duty_interp = interp1d(duty_t, duty_val, kind='linear', fill_value='extrapolate')
duty_interp_vals = duty_interp(odom_t)

# Interpolate servo to mocap time (if exists)
if utils.IS_SERVO:
    servo_t, servo_raw = read_servo()
    delta_raw = servo_to_angle_radians(servo_raw)
    delta_interp = interp1d(servo_t, delta_raw, kind='linear', fill_value='extrapolate')
    delta_vals = delta_interp(odom_t)
else:
     delta_vals = np.zeros_like(odom_t)

print("[INFO] Data loaded and processed.")


# === Plotting ===
def plot_all():
    fig, axs = plt.subplots(4, 1, figsize=(10, 14))
    fig.suptitle('RC Car Data Summary', fontsize=16)

    # 1. x vs y
    axs[0].plot(odom_x, odom_y, label='Trajectory')
    axs[0].set_xlabel('X (m)')
    axs[0].set_ylabel('Y (m)')
    axs[0].set_title('1. X vs Y')
    axs[0].grid(True)
    axs[0].axis('equal')
    axs[0].legend()

    # 2. duty cycle + speed vs time
    axs[1].plot(odom_t, duty_interp_vals, label='Duty Cycle (interpolated)', color='blue')
    axs[1].plot(odom_t, odom_speed, label='Odometry Speed', color='red')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Speed / Duty')
    axs[1].set_title('2. Duty Cycle and Speed vs Time')
    axs[1].grid(True)
    axs[1].legend()

    # 3. yaw vs time
    axs[2].plot(odom_t, odom_yaw, label='Yaw (rad)', color='purple')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Yaw (rad)')
    axs[2].set_title('3. Yaw over Time')
    axs[2].grid(True)
    axs[2].legend()

    # 4. x and y vs time
    axs[3].plot(odom_t, odom_x, label='X Position', color='green')
    axs[3].plot(odom_t, odom_y, label='Y Position', color='orange')
    axs[3].set_xlabel('Time (s)')
    axs[3].set_ylabel('Position (m)')
    axs[3].set_title('4. X and Y vs Time')
    axs[3].grid(True)
    axs[3].legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    plt.show()

    # Optional: plot delta if servo exists
    if utils.IS_SERVO:
        plt.figure()
        plt.plot(odom_t, delta_vals, label='Steering Angle δ (rad)', color='darkcyan')
        plt.xlabel('Time (s)')
        plt.ylabel('Steering Angle (rad)')
        plt.title('Steering Angle vs Time')
        plt.grid(True)
        plt.legend()
        plt.show()


plot_all()
