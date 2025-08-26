
* start carla and carla ros2 bridge
* spawn ego vehicle and activate the traffic manager
    * object definition file: https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_spawn_objects/
```
ros2 launch carla_spawn_objects carla_spawn_objects.launch.py \
    objects_definition_file:=ego_vehicle.json
python3 vehicle_ctrl.py
```
* start recording the IMU data (see ros2 topic list)
```
ros2 bag record /carla/ego_vehicle/imu /carla/ego_vehicle/rgb_view/image \
    /carla/ego_vehicle/rgb_view/camera_info
```
* record for a while then stop by stopping the script above