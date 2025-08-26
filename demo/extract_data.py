import rclpy
import pandas as pd
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

storage_options = StorageOptions(uri='./raw_demo_data/raw_demo_data.db3', storage_id='sqlite3')
converter_options = ConverterOptions('', '')

reader = SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topic_types}

linear_acc = []
angular_velocity = []
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == '/carla/ego_vehicle/imu':
        imu_msg = deserialize_message(data, Imu)

        linear_acc.append((
            imu_msg.header.stamp.sec + 1e-9 * imu_msg.header.stamp.nanosec,
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ))

        angular_velocity.append((
            imu_msg.header.stamp.sec + 1e-9 * imu_msg.header.stamp.nanosec,
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
        ))

linear_acc = pd.DataFrame(linear_acc, columns=["sec", "linear_acc_x","linear_acc_y","linear_acc_z"])
linear_acc.to_csv("linear_acc.csv", index=False)
angular_velocity = pd.DataFrame(angular_velocity, columns= ["sec", "angular_velocity_x", "angular_velocity_y", "angular_velocity_z"])
angular_velocity.to_csv("angular_velocity.csv", index=False)

ax = linear_acc.plot(x='sec')
ax.figure.savefig("linear_acc.png")
ax = angular_velocity.plot(x='sec')
ax.figure.savefig("angular_velocity.png")