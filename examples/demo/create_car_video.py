import cv2
from cv_bridge import CvBridge
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
import rclpy.serialization

# Parameters
bag_path = "./raw_demo_data/raw_demo_data.db3"
topic_name = "/carla/ego_vehicle/rgb_view/image"
output_video = "carla_camera.mp4"
fps = 25

bridge = CvBridge()

# Setup ROS 2 bag reader
storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
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
            video_writer = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

        video_writer.write(cv_image)
        frame_count += 1

if video_writer:
    video_writer.release()

print(f"MP4 video created: {output_video}, total frames: {frame_count}")