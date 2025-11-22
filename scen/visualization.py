import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
import numpy as np
import h5py
import gc
for obj in gc.get_objects():   # Browse through ALL objects
    if isinstance(obj, h5py.File):   # Just HDF5 files
        try:
            obj.close()
        except:
            pass # Was already closed

to_type = {
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
to_id = {v: k for k, v in to_type.items()}

def create_pcl_video(pcl_points, output_file,bbox_file, topic, semantic=True):
    with h5py.File(pcl_points, 'r') as f, h5py.File(bbox_file, 'r') as f_bbox:
        print(f"reading file {pcl_points}...")

        fig = plt.figure(figsize=(10,8))
        ax = fig.add_subplot(111, projection='3d')

        # Set axes limits (adjust based on your LiDAR range)
        #ax.view_init(elev=20, azim=180)
        ax.set_xlim(-50, 50)
        ax.set_ylim(-50, 50)
        ax.set_zlim(-5, 5)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title('LiDAR 3D Top-Down View')

        # Initialize scatter (empty)
        scat = ax.scatter([], [], [], c=[], s=1, cmap='viridis')

        # Update function for animation
        def update(frame_idx):
            try:
                ax.cla()  # clear previous points
                points = f[f"{topic}/frame_{frame_idx:06d}"]
                boxes = list(f_bbox[f"frame_{frame_idx:06d}/actors"])

                x = points["x"]
                y = points["y"]
                z = points["z"]
                i = [to_id[point.decode('utf8')] for point in points["ObjTag"]]

                ax.scatter(x, y, z, c=i, s=1, cmap='viridis')

                #plot bounding boxes no rotation needed
                for box in boxes:
                    
                    id, bx, by, bz, bxextend, byextend, bzextend, broll, bpitch, byaw = box
                    
                    #get points of the box
                    points = np.array([
                        [bx - bxextend, by - byextend, bz - bzextend],
                        [bx + bxextend, by - byextend, bz - bzextend],
                        [bx + bxextend, by + byextend, bz - bzextend],
                        [bx - bxextend, by + byextend, bz - bzextend],
                        [bx - bxextend, by - byextend, bz + bzextend],
                        [bx + bxextend, by - byextend, bz + bzextend],
                        [bx + bxextend, by + byextend, bz + bzextend],
                        [bx - bxextend, by + byextend, bz + bzextend],
                    ])
                    #draw lines between the points
                    lines = [
                        [points[0], points[1]],
                        [points[1], points[2]],
                        [points[2], points[3]],
                        [points[3], points[0]],
                        [points[4], points[5]],
                        [points[5], points[6]],
                        [points[6], points[7]],
                        [points[7], points[4]],
                        [points[0], points[4]],
                        [points[1], points[5]],
                        [points[2], points[6]],
                        [points[3], points[7]],
                    ]
                    for line in lines:
                        ax.plot([line[0][0], line[1][0]],
                                [line[0][1], line[1][1]],
                                [line[0][2], line[1][2]], c='r', alpha=0.5)
                    
                
                ax.set_xlim(-10,10)
                ax.set_ylim(-10,10)
                ax.set_zlim(-10,10)
                ax.set_xlabel('X [m]')
                ax.set_ylabel('Y [m]')
                ax.set_zlabel('Z [m]')
                ax.set_title(f"LiDAR Frame {frame_idx}")
                return []
            except KeyError:
                print(f"Frame {frame_idx} not found in the dataset.")
                return []

        print("Create animation...")
        ani = FuncAnimation(fig, update, frames=len(f_bbox.keys())-1, interval=50)

        print("Save as MP4...")
        ani.save(output_file, writer='ffmpeg', fps=20)
        plt.close(fig)

import cv2
from cv_bridge import CvBridge
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
import rclpy.serialization

def create_camera_video(db_file, topic_name, output_file, fps=20):
    bridge = CvBridge()

    # Setup ROS 2 bag reader
    storage_options = StorageOptions(uri=db_file, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Get all topics
    topic_types = reader.get_all_topics_and_types()
    topics_dict = {t.name: t.type for t in topic_types}

    video_writer = None
    frame_count = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            # Deserialize message
            img_msg = rclpy.serialization.deserialize_message(data, Image)
            cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            if video_writer is None:
                height, width, _ = cv_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

            video_writer.write(cv_image)
            frame_count += 1
    
    if video_writer:
        video_writer.release()

    print(f"MP4 video created: {output_file}, total frames: {frame_count}")

if __name__ == '__main__':
    
    db_dir = Path('/home/npopkov/repos/IR2025/data/251119_eight_lidar_10s/db/')
    bbox_dir = Path('/home/npopkov/repos/IR2025/data/251119_eight_lidar_10s/')
    create_pcl_video(
        str(db_dir / "lidar_data.h5"),
        str(db_dir / "lidar_3d_video.mp4"),
        str(bbox_dir / "bbox.h5"),
        "_carla_ego_vehicle_lidar_front"
    )
    # create_camera_video(
    #     db_file = str(db_dir / 'rosbag2_2025_10_11-19_24_30_0.db3'),
    #     topic_name = "/carla/ego_vehicle/rgb_view/image",
    #     output_file = str(db_dir / "carla_camera.mp4")
    # )