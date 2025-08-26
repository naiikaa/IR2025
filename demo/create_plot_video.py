import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def create_evolving_plot(csv_fname, title, y_label="Value", time_col_name="sec", fps=25):
    # Load CSV
    df = pd.read_csv(csv_fname)
    df[time_col_name] = df[time_col_name] - df[time_col_name][0]
    df_cols = df.columns.copy()
    df_cols = df_cols.drop(time_col_name)

    fig, ax = plt.subplots()
    lines = []
    colors = ['blue', 'red', 'green']
    labels = df_cols

    for color, label in zip(colors, labels):
        line, = ax.plot([], [], lw=2, color=color, label=label)
        lines.append(line)

    ax.set_xlim(df[time_col_name].min(), df[time_col_name].max())
    all_values = df[df_cols].values
    ax.set_ylim(all_values.min() - 1, all_values.max() + 1)
    ax.set_xlabel('Time in s')
    ax.set_ylabel(y_label)
    ax.set_title(title)
    ax.legend()

    # Initialization function
    def init():
        for line in lines:
            line.set_data([], [])
        return lines

    # Update function
    def update(frame):
        x = df[time_col_name][:frame+1]
        for i, line in enumerate(lines):
            y = df.iloc[:frame+1, i+1]  # +1 because first column is time
            line.set_data(x, y)
        return lines

    # Create animation
    ani = FuncAnimation(fig, update, frames=len(df), init_func=init, blit=True, interval=(1000 / fps))

    # Save as MP4
    ani.save(f'{csv_fname.split(".")[0]}.mp4', writer='ffmpeg', fps=fps)

if __name__ == '__main__':
    create_evolving_plot("angular_velocity.csv", "Angular Velocity", "")
    create_evolving_plot("linear_acc.csv", "Linear Acceleration", "")