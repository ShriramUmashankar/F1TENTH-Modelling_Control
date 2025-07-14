import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import pandas as pd
import utils
import os

# Set IS_SERVO based on topic presence in bag
utils.check_servo_topic_exists()

def read_ros2_topic(bag_path, topic_name, msg_type):
    """Read messages from a ROS 2 bag topic and return as a DataFrame."""
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    available_topics = [info.name for info in reader.get_all_topics_and_types()]
    if topic_name not in available_topics:
        print(f"[WARNING] Topic '{topic_name}' not found in bag.")
        return None

    reader.set_filter(StorageFilter(topics=[topic_name]))
    messages = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, msg_type)
        timestamp = t * 1e-9  # Convert nanoseconds to seconds

        if topic == '/commands/motor/duty_cycle':
            messages.append({'timestamp': timestamp, 'duty_cycle': msg.data})

        elif topic == '/commands/servo/position':
            messages.append({'timestamp': timestamp, 'servo_angle': msg.data})

        elif topic == '/mocap/odom':
            messages.append({
                'timestamp': timestamp,
                'position_x': msg.pose.pose.position.x,
                'position_y': msg.pose.pose.position.y,
                'position_z': msg.pose.pose.position.z,
                'orientation_x': msg.pose.pose.orientation.x,
                'orientation_y': msg.pose.pose.orientation.y,
                'orientation_z': msg.pose.pose.orientation.z,
                'orientation_w': msg.pose.pose.orientation.w,
                'velocity_x': msg.twist.twist.linear.x,
                'velocity_y': msg.twist.twist.linear.y,
                'angular_z': msg.twist.twist.angular.z,
            })

    return pd.DataFrame(messages)


def save_csv(df, filename):
    if df is not None and not df.empty:
        df.to_csv(filename, index=False)
        print(f"[SAVED] {os.path.basename(filename)}")
    else:
        print(f"[SKIPPED] No data to save for {os.path.basename(filename)}")


def main():
    rclpy.init()

    bag_path = utils.BAG_PATH
    save_path = utils.PATH

    df_duty = read_ros2_topic(bag_path, '/commands/motor/duty_cycle', Float64)
    save_csv(df_duty, os.path.join(save_path, 'duty.csv'))

    if utils.IS_SERVO:
        df_servo = read_ros2_topic(bag_path, '/commands/servo/position', Float64)
        save_csv(df_servo, os.path.join(save_path, 'servo.csv'))
    else:
        print("[INFO] /commands/servo/position topic not present. Skipping.")

    df_odom = read_ros2_topic(bag_path, '/mocap/odom', Odometry)
    save_csv(df_odom, os.path.join(save_path, 'odom.csv'))

    rclpy.shutdown()


if __name__ == '__main__':
    main()
