import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from networktables import NetworkTables

# =========================================================
# CONFIGURATION
# =========================================================

ROBOT_IP = "10.23.45.2"

MAX_DISTANCE_METERS = 5.0

MAP_SIZE_METERS = 10.0
RESOLUTION = 0.05
GRID_SIZE = int(MAP_SIZE_METERS / RESOLUTION)

UNKNOWN = -1
FREE = 0
OCCUPIED = 1

data_lock = threading.Lock()

# =========================================================
# ROBOT POSE
# =========================================================
# For now robot is fixed.
# Later these values will come from odometry / IMU.

robot_x = 0.0
robot_y = 0.0
robot_heading_deg = 0.0

# =========================================================
# LIVE LIDAR DATA
# =========================================================

latest_angles = []
latest_distances = []

# =========================================================
# ACCUMULATED MAP POINTS
# =========================================================

map_x_points = []
map_y_points = []

MAX_MAP_POINTS = 10000

# =========================================================
# OCCUPANCY GRID
# =========================================================

grid = np.full(
    (GRID_SIZE, GRID_SIZE),
    UNKNOWN,
    dtype=np.int8
)

# =========================================================
# WORLD TO GRID
# =========================================================

def world_to_grid(x, y):

    origin = GRID_SIZE // 2

    gx = int(x / RESOLUTION) + origin
    gy = int(y / RESOLUTION) + origin

    return gx, gy

# =========================================================
# BRESENHAM LINE
# =========================================================

def bresenham(x0, y0, x1, y1):

    points = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    x = x0
    y = y0

    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    if dx > dy:

        err = dx / 2

        while x != x1:

            points.append((x, y))

            err -= dy

            if err < 0:
                y += sy
                err += dx

            x += sx

    else:

        err = dy / 2

        while y != y1:

            points.append((x, y))

            err -= dx

            if err < 0:
                x += sx
                err += dy

            y += sy

    points.append((x1, y1))

    return points

# =========================================================
# CONVERT LIDAR POINT TO WORLD COORDINATES
# =========================================================

def lidar_to_world(angle_deg, distance_mm):

    distance_m = distance_mm / 1000.0

    global_angle_deg = (
        robot_heading_deg + angle_deg
    )

    global_angle_rad = np.radians(
        global_angle_deg
    )

    hit_x = (
        robot_x +
        distance_m * np.cos(global_angle_rad)
    )

    hit_y = (
        robot_y +
        distance_m * np.sin(global_angle_rad)
    )

    return hit_x, hit_y

# =========================================================
# UPDATE OCCUPANCY GRID
# =========================================================

def update_grid_point(angle_deg, distance_mm):

    if distance_mm < 120:
        return

    if distance_mm > 5000:
        return

    hit_x, hit_y = lidar_to_world(
        angle_deg,
        distance_mm
    )

    robot_gx, robot_gy = world_to_grid(
        robot_x,
        robot_y
    )

    hit_gx, hit_gy = world_to_grid(
        hit_x,
        hit_y
    )

    if not (
        0 <= hit_gx < GRID_SIZE and
        0 <= hit_gy < GRID_SIZE
    ):
        return

    ray = bresenham(
        robot_gx,
        robot_gy,
        hit_gx,
        hit_gy
    )

    # Mark cells before obstacle as free
    for gx, gy in ray[:-1]:

        if (
            0 <= gx < GRID_SIZE and
            0 <= gy < GRID_SIZE
        ):
            grid[gy, gx] = FREE

    # Mark obstacle
    grid[hit_gy, hit_gx] = OCCUPIED

# =========================================================
# NETWORKTABLES CALLBACK
# =========================================================

def lidar_callback(table, key, value, isNew):

    global latest_angles
    global latest_distances
    global map_x_points
    global map_y_points

    if not key.startswith("Scan"):
        return

    angles = value[0::2]
    distances = value[1::2]

    length = min(
        len(angles),
        len(distances)
    )

    new_angles = []
    new_distances = []

    with data_lock:

        for i in range(length):

            angle = float(angles[i])
            distance = float(distances[i])

            if 120 <= distance <= 5000:

                # -----------------------------
                # LIVE RADAR DATA
                # -----------------------------

                new_angles.append(angle)
                new_distances.append(distance)

                # -----------------------------
                # OCCUPANCY GRID
                # -----------------------------

                update_grid_point(
                    angle,
                    distance
                )

                # -----------------------------
                # ACCUMULATED MAP
                # -----------------------------

                hit_x, hit_y = lidar_to_world(
                    angle,
                    distance
                )

                map_x_points.append(hit_x)
                map_y_points.append(hit_y)

        latest_angles = new_angles
        latest_distances = new_distances

        # Prevent unlimited memory growth
        if len(map_x_points) > MAX_MAP_POINTS:

            map_x_points = map_x_points[-MAX_MAP_POINTS:]
            map_y_points = map_y_points[-MAX_MAP_POINTS:]

# =========================================================
# DISPLAY UPDATE
# =========================================================

def update_display(frame):

    with data_lock:

        angles = np.array(
            latest_angles
        )

        distances = np.array(
            latest_distances
        )

        grid_copy = grid.copy()

        map_x = np.array(
            map_x_points
        )

        map_y = np.array(
            map_y_points
        )

    # =====================================================
    # VIEW 1 - LIVE RADAR
    # =====================================================

    radar_ax.clear()

    radar_ax.set_theta_zero_location("N")
    radar_ax.set_theta_direction(-1)

    radar_ax.set_ylim(
        0,
        MAX_DISTANCE_METERS
    )

    radar_ax.set_title(
        "1. Live LiDAR Radar"
    )

    if len(angles) > 0:

        angles_rad = np.radians(
            angles
        )

        distances_m = (
            distances / 1000.0
        )

        # Laser rays
        for angle, distance in zip(
            angles_rad,
            distances_m
        ):

            radar_ax.plot(
                [angle, angle],
                [0, distance],
                linewidth=0.4,
                alpha=0.2
            )

        # Detection points
        radar_ax.scatter(
            angles_rad,
            distances_m,
            s=12
        )

    # Robot centre
    radar_ax.scatter(
        [0],
        [0],
        s=80
    )

    # =====================================================
    # VIEW 2 - OCCUPANCY GRID
    # =====================================================

    grid_image.set_data(
        grid_copy
    )

    # =====================================================
    # VIEW 3 - ACCUMULATED MAP
    # =====================================================

    map_ax.clear()

    map_ax.set_title(
        "3. Accumulated LiDAR Map"
    )

    map_ax.set_xlabel(
        "X (m)"
    )

    map_ax.set_ylabel(
        "Y (m)"
    )

    map_ax.set_xlim(
        -MAP_SIZE_METERS / 2,
        MAP_SIZE_METERS / 2
    )

    map_ax.set_ylim(
        -MAP_SIZE_METERS / 2,
        MAP_SIZE_METERS / 2
    )

    map_ax.set_aspect(
        "equal"
    )

    map_ax.grid(
        True
    )

    if len(map_x) > 0:

        map_ax.scatter(
            map_x,
            map_y,
            s=3
        )

    # Robot position
    map_ax.scatter(
        [robot_x],
        [robot_y],
        s=80
    )

    return grid_image,

# =========================================================
# MAIN
# =========================================================

def main():

    print(
        f"Connecting to robot at {ROBOT_IP}..."
    )

    NetworkTables.initialize(
        server=ROBOT_IP
    )

    NetworkTables.setUpdateRate(
        0.010
    )

    time.sleep(2)

    print(
        "Connected:",
        NetworkTables.isConnected()
    )

    lidar_table = NetworkTables.getTable(
        "Lidar"
    )

    lidar_table.addEntryListener(
        lidar_callback
    )

    # =====================================================
    # CREATE WINDOW WITH 3 VIEWS
    # =====================================================

    global radar_ax
    global grid_ax
    global map_ax
    global grid_image

    fig = plt.figure(
        figsize=(18, 6)
    )

    # 1. Radar
    radar_ax = fig.add_subplot(
        1,
        3,
        1,
        projection="polar"
    )

    # 2. Occupancy grid
    grid_ax = fig.add_subplot(
        1,
        3,
        2
    )

    # 3. Accumulated map
    map_ax = fig.add_subplot(
        1,
        3,
        3
    )

    grid_image = grid_ax.imshow(
        grid,
        origin="lower",
        vmin=-1,
        vmax=1
    )

    grid_ax.set_title(
        "2. Occupancy Grid"
    )

    grid_ax.set_xlabel(
        "Grid X"
    )

    grid_ax.set_ylabel(
        "Grid Y"
    )

    centre = GRID_SIZE // 2

    grid_ax.plot(
        centre,
        centre,
        "ro"
    )

    ani = FuncAnimation(
        fig,
        update_display,
        interval=100,
        cache_frame_data=False
    )

    plt.tight_layout()

    print(
        "Launching radar + grid + map..."
    )

    plt.show()


if __name__ == "__main__":
    main()