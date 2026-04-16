import numpy as np
import matplotlib.pyplot as plt

def plot_lidar(file_path):
    # Load data
    data = np.load(file_path)
    
    # Preparation
    angles = np.linspace(0, np.pi, len(data), endpoint=False)
    
    # Handle INF values for visualization
    plot_data = data.copy()
    plot_data[np.isinf(plot_data)] = np.nan
    
    # Convert angles to degrees for the x-axis
    angles_deg = np.rad2deg(angles)
    
    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(111)
    
    # Plotting
    ax.plot(angles_deg, plot_data, 'r-', linewidth=2)
    
    ax.set_title("1D LIDAR output (Mesh in Dust)", fontsize=15)
    ax.set_xlabel("Angle (Degrees)", fontsize=12)
    ax.set_ylabel("Distance (m)", fontsize=12)
    ax.set_xlim(0, 180)
    ax.set_ylim(0, 5)
    ax.grid(True, linestyle='--', alpha=0.7)

    save_path = "lidar_line_plot_2128.png"
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Plot saved to: {save_path}")

if __name__ == "__main__":
    plot_lidar('lidar_2128.npy')
