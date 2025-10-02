use recording like in the demo example
to visualize, use rviz2 -> set fixed_frame to ego_vehicle/lidar, add pointcloud2 display time, add topic
* start carla and ros bridge
* spawn ego vehicle
```
ros2 launch carla_spawn_objects carla_spawn_objects.launch.py \
    objects_definition_file:=ego_vehicle.json
python3 vehicle_ctrl.py
```
* start recording
```
ros2 bag record /carla/ego_vehicle/lidar
```

* start vehicle_ctrl to see movement
* use extract_lidar_data to convert into .npy
* use plot_pointcloud_frame to create video