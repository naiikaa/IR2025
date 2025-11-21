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