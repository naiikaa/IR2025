lidar_data.hdf5
    sensors
        frames
            points (x, y, z, cos, objidx, objtag)
    metadaten (todo)

bbox.hdf5
    frames
        ego
            (x,y,z, x_extent,y_extent,z_extent,roll,pitch,yaw)
        actors
            (actor_id, x,y,z, x_extent,y_extent,z_extent,roll,pitch,yaw)
            (actor_id, x,y,z, x_extent,y_extent,z_extent,roll,pitch,yaw)
            (actor_id, x,y,z, x_extent,y_extent,z_extent,roll,pitch,yaw)
    actor_types (todo)
        (actor_id, actor_type)
        (actor_id, actor_type)
        (actor_id, actor_type)

todo:
coord-sys:
    ego_vehicle
bbox format:
    normalerweise xyz, extents, nur 1 rotation
funktion
    umrechnung von coord-sys der sensoren <=> ego_vehicle
readme
    erklärungen zu cos, objtag
    beispiel zur struktur der hdf5
    sensor setup (wo platziert, richtung, winkel etc., coord-sys, units)