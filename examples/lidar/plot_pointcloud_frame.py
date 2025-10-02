import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

frames = np.load("lidar_points.npy", allow_pickle=True)
frames = frames[600:800]

fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111, projection='3d')

# Set axes limits (adjust based on your LiDAR range)
ax.view_init(elev=20, azim=180)
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
    ax.cla()  # clear previous points
    points = frames[frame_idx]

    x = points["x"]
    y = points["y"]
    z = points["z"]
    i = points["intensity"]

    ax.scatter(x, y, z, c=i, s=1, cmap='viridis')
    
    ax.set_xlim(-50,50)
    ax.set_ylim(-50,50)
    ax.set_zlim(-5,5)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title(f"LiDAR Frame {frame_idx}")
    return []

# Create animation
ani = FuncAnimation(fig, update, frames=len(frames), interval=50)

# Save as MP4 (requires ffmpeg)
ani.save("lidar_3d_video.mp4", writer='ffmpeg', fps=20)
plt.close(fig)