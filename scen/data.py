import pandas as pd
import numpy as np
import os, h5py, yaml
from pathlib import Path

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import cv2
from cv_bridge import CvBridge

semantic_lidar_tags = {
  0 : "Unlabeled",
  1 : "Roads",
  2 : "SideWalks",
  3 : "Buildings",
  4 : "Wall",
  5 : "Fence",
  6 : "Pole",
  7 : "TrafficLight",
  8 : "TrafficSign",
  9 : "Vegetation",
  10 : "Terrain",
  11 : "Sky",
  12 : "Pedestrian",
  13 : "Rider",
  14 : "Car",
  15 : "Truck",
  16 : "Bus",
  17 : "Train",
  18 : "Motorcycle",
  19 : "Bicycle",
  20 : "Static",
  21 : "Dynamic",
  22 : "Other",
  23 : "Water",
  24 : "RoadLine",
  25 : "Ground",
  26 : "Bridge",
  27 : "RailTrack",
  28 : "GuardRail", 
}

def semantic_tag_to_id(points):
    new_dtype = [(n, "u4" if n == 'ObjTag' else points.dtype[n])
             for n in points.dtype.names]

    new_points = np.empty(points.shape, dtype=new_dtype)

    for name in points.dtype.names:
        if name != "ObjTag":
            new_points[name] = points[name]

    categories = [s.encode("utf8") for s in semantic_lidar_tags.values()]
    cat = pd.Categorical(points["ObjTag"], categories=categories)

    new_points["ObjTag"] = cat.codes
    return new_points

def semantic_id_to_tag(points):
    lookup = np.array(list(semantic_lidar_tags.values()))
    new_dtype = [(n, h5py.string_dtype(encoding='utf-8') if n == 'ObjTag' else points.dtype[n]) for n in points.dtype.names]
    
    new_points = points.astype(new_dtype)
    new_points[:]["ObjTag"] = lookup[points[:]["ObjTag"]]
    return new_points

def save_metadata(lidar_data_fpath, car_config):
    with open(car_config, "r") as f:
        data = f.read()
    with h5py.File(lidar_data_fpath, "a") as f:
        f["metadata2"] = [data]
        #f.create_dataset("metadata", data=data)

def extract_pcl_data(dbfile, pcl_topics: list, output):
    storage_options = StorageOptions(uri=dbfile, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    h5_file = h5py.File(output, 'w')

    topic_groups = {topic: h5_file.create_group(topic.replace('/', '_')) for topic in pcl_topics}
    frame_counters = {topic: 0 for topic in pcl_topics}
    
    frame_idx = 0
    while reader.has_next():
        print(f"Processing frame {frame_idx}...\r", end="",flush=True)
        topic, data, t = reader.read_next()
        if topic not in pcl_topics:
            continue
        msg = deserialize_message(data, PointCloud2)

        # Extract points as numpy array
        field_names = [field.name for field in msg.fields]
        points = pc2.read_points(msg, field_names=field_names, skip_nans=True)
        
        # Transform objtag id into tag 
        new_points = semantic_id_to_tag(points)
        frame_idx = frame_counters[topic]

        topic_groups[topic].create_dataset(
            f"frame_{frame_idx:06d}",
            data=new_points,
            compression='gzip',
            compression_opts=4
        )

        frame_counters[topic] += 1
    h5_file.close()

def extract_camera_data(db_file, topic_name, output_dir):
    bridge = CvBridge()
    os.makedirs(output_dir, exist_ok=True)

    # Setup ROS 2 bag reader
    storage_options = StorageOptions(uri=db_file, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    frame_count = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            img_msg = deserialize_message(data, Image)
            cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            cv2.imwrite(str(Path(output_dir) / f"frame{frame_count}.png"), cv_image)
            frame_count += 1
    

    print(f"Camera data extracted, total frames: {frame_count}")

def extract_pcl_topics(metadata_fpath):
    with open(metadata_fpath, 'r') as f:
        metadata = yaml.safe_load(f)

    topic_metadata = metadata["rosbag2_bagfile_information"]["topics_with_message_count"]

    topic_list = [topic["topic_metadata"]["name"] for topic in topic_metadata if topic["topic_metadata"]["type"] == "sensor_msgs/msg/PointCloud2"]
    return topic_list

if __name__ == '__main__':
    db_dir = Path('/home/npopkov/repos/IR2025/data/251119_eight_lidar_10s/db/')
    car_dir = Path('/home/npopkov/repos/IR2025/data/251119_eight_lidar_10s/')
    topic_list = extract_pcl_topics(db_dir / "metadata.yaml")
    extract_pcl_data(
        str(db_dir / 'db_0.db3'),
        topic_list,
        str(db_dir / 'lidar_data.h5')
    )
    save_metadata(
        str(db_dir / 'lidar_data.h5'),
        str(car_dir / 'eight_car_lidar.json')
    )
    # extract_camera_data(
    #     db_file = str(db_dir / 'rosbag2_2025_10_11-19_24_30_0.db3'),
    #     topic_name = "/carla/ego_vehicle/rgb_view/image",
    #     output_dir = str(db_dir / "carla_camera")
    # )