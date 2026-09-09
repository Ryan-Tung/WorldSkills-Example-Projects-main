import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from networktables import NetworkTables
from collections import deque

import logging
logging.basicConfig(level=logging.WARNING)  

# --- CONFIGURATION ---
ROBOT_IP = '10.23.45.2' # IMPORTANT: Change this to your robot's IP
MAX_DISTANCE_METERS = 5.0

data_lock = threading.Lock()

# --- CONTINUOUS ROLLING BUFFER ---
# 500 points easily holds the ~360 points your Lidar is currently outputting
HISTORY_LENGTH = 500
rolling_angles = deque(maxlen=HISTORY_LENGTH)
rolling_distances = deque(maxlen=HISTORY_LENGTH)

def value_changed_callback(table, key, value, isNew):
    """
    Listens for ANY chunk starting with 'Scan' and pushes it to the rolling map.
    """
    global rolling_angles, rolling_distances
    
    if key.startswith('Scan'):
        with data_lock:
            new_angles = value[0::2]
            new_distances = value[1::2]
            
            min_len = min(len(new_angles), len(new_distances))
            
            # Append the chunk to the back of the queue
            rolling_angles.extend(new_angles[:min_len])
            rolling_distances.extend(new_distances[:min_len])

def update_plot(frame):
    """
    Animation loop that updates the radar plot with accumulated data.
    """
    with data_lock:
        if len(rolling_angles) == 0:
            return scatter,
        
        angles_deg = np.array(rolling_angles)
        distances_mm = np.array(rolling_distances)
        
    distances_m = distances_mm / 1000.0
    angles_rad = np.radians(angles_deg)
    
    # Filter out zero distances (invalid readings)
    valid_points = distances_m > 0.05 
    
    # Plot the clean, rolling 360 map
    scatter.set_offsets(np.c_[angles_rad[valid_points], distances_m[valid_points]])
    
    return scatter,

def main():
    print(f"Connecting to NetworkTables at {ROBOT_IP}...")
    NetworkTables.initialize(server=ROBOT_IP)
    NetworkTables.setUpdateRate(0.010)
    
    lidar_table = NetworkTables.getTable("Lidar")
    lidar_table.addEntryListener(value_changed_callback)

    # --- SETUP THE RADAR PLOT ---
    global scatter
    fig = plt.figure(figsize=(8, 8))
    
    ax = fig.add_subplot(111, projection='polar')
    ax.set_title("Live Studica Lidar Stream (Full 360 Map)", va='bottom', color='white')
    
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_ylim(0, MAX_DISTANCE_METERS)
    
    scatter = ax.scatter([], [], c='#39FF14', s=15, alpha=0.75, edgecolors='black')
    
    fig.patch.set_facecolor('#1e1e1e')
    ax.set_facecolor('#000000')
    ax.tick_params(colors='white')
    ax.grid(color='#333333')
    ax.spines['polar'].set_color('#555555')

    ani = FuncAnimation(fig, update_plot, interval=50, blit=True, save_count=50)
    
    print("Launching full 360 radar viewer...")
    plt.show()

if __name__ == "__main__":
    main()