import numpy as np

def coords_to_ego(bbox_worldcord, ego_vehicle_worldcord):
    # Convert bbox from world coordinates to ego vehicle coordinates
    def rotation_matrix(roll, pitch, yaw):
        roll = roll * np.pi / 180
        pitch = pitch * np.pi / 180
        yaw = yaw * np.pi / 180

        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        
        return Rz @ Ry @ Rx 
    Re = rotation_matrix(ego_vehicle_worldcord[3], ego_vehicle_worldcord[4], ego_vehicle_worldcord[5])

    Rw = rotation_matrix(bbox_worldcord[3], bbox_worldcord[4], bbox_worldcord[5])
    
    Rbe = Re.T @ Rw

    bbox_ego_coords = Re.T @ (np.array(bbox_worldcord[0:3]) - np.array(ego_vehicle_worldcord[0:3]))
    bbox_ego_rotation = np.array([
        np.arctan2(Rbe[2,1], Rbe[2,2]),
        np.arcsin(-Rbe[2,0]),
        np.arctan2(Rbe[1,0], Rbe[0,0])]
    )

    bbox_ego_rotation = bbox_ego_rotation * 180 / np.pi

    return bbox_ego_coords, bbox_ego_rotation

def coords_to_ego_vectorized(bbox_worldcord, ego_vehicle_worldcord):
    """bbox_worldcord should be of (N, 6)
    """
    # Convert bbox from world coordinates to ego vehicle coordinates
    bbox_worldcord = np.asarray(bbox_worldcord)
    ego_vehicle_worldcord = np.asarray(ego_vehicle_worldcord)
    
    def rotation_matrix(roll, pitch, yaw):
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        Rx = np.stack([
            np.stack([np.ones_like(roll), 0*roll, 0*roll], axis=1),
            np.stack([0*roll, cr, -sr], axis=1),
            np.stack([0*roll, sr, cr], axis=1)
        ], axis=1)
        
        Ry = np.stack([
            np.stack([cp, 0*pitch, sp], axis=1),
            np.stack([0*pitch, np.ones_like(pitch), 0*pitch], axis=1),
            np.stack([-sp, 0*pitch, cp], axis=1)
        ], axis=1)
        
        Rz = np.stack([
            np.stack([cy, -sy, 0*yaw], axis=1),
            np.stack([sy, cy, 0*yaw], axis=1),
            np.stack([0*yaw, 0*yaw, np.ones_like(yaw)], axis=1)
        ], axis=1)
        
        return Rz @ Ry @ Rx 
    
    Re = rotation_matrix(
        np.array([ego_vehicle_worldcord[3]]),
        np.array([ego_vehicle_worldcord[4]]),
        np.array([ego_vehicle_worldcord[5]])
    )[0]

    Rw = rotation_matrix(
        bbox_worldcord[:, 3], 
        bbox_worldcord[:, 4], 
        bbox_worldcord[:, 5]
    )

    Rbe = Re.T @ Rw
    roll = np.arctan2(Rbe[2,1], Rbe[2,2])
    pitch = np.arcsin(-Rbe[2,0])
    yaw = np.arctan2(Rbe[1,0], Rbe[0,0])

    bbox_ego_coords = (Re.T @ (bbox_worldcord[:, :3] - ego_vehicle_worldcord[:3]).T).T

    bbox_ego_rotation = np.stack([roll, pitch, yaw], axis=-1)
    bbox_ego_rotation = np.rad2deg(bbox_ego_rotation)

    return bbox_ego_coords, bbox_ego_rotation