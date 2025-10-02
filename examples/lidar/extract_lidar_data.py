import rclpy
import pandas as pd
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import sensor_msgs_py.point_cloud2 as pc2


storage_options = StorageOptions(uri='./raw_data/raw_data.db3', storage_id='sqlite3')
converter_options = ConverterOptions('', '')

reader = SequentialReader()
reader.open(storage_options, converter_options)

lidar_topic = "/carla/ego_vehicle/lidar"
all_points = []

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic != lidar_topic:
        continue
    msg = deserialize_message(data, PointCloud2)
    # Extract points as numpy array (x, y, z, intensity)
    points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)))
    all_points.append(points)

a = np.array(all_points, dtype=object)
np.save("lidar_points.npy", a)