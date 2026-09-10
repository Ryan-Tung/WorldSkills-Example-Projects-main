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

robot_x = 0.0
robot_y = 0.0
robot_heading_deg = 0.0


# =========================================================
# LIVE LIDAR DATA
# =========================================================

scan_a_angles = []
scan_a_distances = []

scan_b_angles = []
scan_b_distances = []


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
# GET ROBOT POSE FROM JAVA
# =========================================================

def update_robot_pose():

    global robot_x
    global robot_y
    global robot_heading_deg

    robot_x = (
        pose_table
        .getEntry("X")
        .getDouble(robot_x)
    )

    robot_y = (
        pose_table
        .getEntry("Y")
        .getDouble(robot_y)
    )

    robot_heading_deg = (
        pose_table
        .getEntry("Heading")
        .getDouble(robot_heading_deg)
    )


# =========================================================
# CONVERT LIDAR POINT TO WORLD COORDINATES
# =========================================================

def lidar_to_world(angle_deg, distance_mm):

    distance_m = distance_mm / 1000.0

    # -----------------------------------------------------
    # Robot convention:
    #
    # 0   = front
    # 90  = right
    # 180 = back
    # 270 = left
    # -----------------------------------------------------

    global_angle_deg = (
        robot_heading_deg
        +
        angle_deg
    )

    global_angle_rad = np.radians(
        global_angle_deg
    )


    # X = right / left
    # Y = forward / back

    hit_x = (
        robot_x
        +
        distance_m
        *
        np.sin(global_angle_rad)
    )

    hit_y = (
        robot_y
        +
        distance_m
        *
        np.cos(global_angle_rad)
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
        0 <= hit_gx < GRID_SIZE
        and
        0 <= hit_gy < GRID_SIZE
    ):
        return


    ray = bresenham(
        robot_gx,
        robot_gy,
        hit_gx,
        hit_gy
    )


    # -----------------------------------------------------
    # Everything before obstacle = free
    # -----------------------------------------------------

    for gx, gy in ray[:-1]:

        if (
            0 <= gx < GRID_SIZE
            and
            0 <= gy < GRID_SIZE
        ):
            grid[gy, gx] = FREE


    # -----------------------------------------------------
    # Final point = obstacle
    # -----------------------------------------------------

    grid[hit_gy, hit_gx] = OCCUPIED


# =========================================================
# PROCESS ONE LIDAR ARRAY
# =========================================================

def process_scan(value):

    angles = value[0::2]
    distances = value[1::2]

    length = min(
        len(angles),
        len(distances)
    )

    new_angles = []
    new_distances = []


    update_robot_pose()


    for i in range(length):

        angle = float(
            angles[i]
        )

        distance = float(
            distances[i]
        )


        if 120 <= distance <= 5000:

            new_angles.append(
                angle
            )

            new_distances.append(
                distance
            )


            # =================================================
            # OCCUPANCY GRID ONLY
            # =================================================
            #
            # View 2 still keeps mapping history.
            #
            # View 3 DOES NOT store old points anymore.

            update_grid_point(
                angle,
                distance
            )


    return (
        new_angles,
        new_distances
    )


# =========================================================
# NETWORKTABLES CALLBACK
# =========================================================

def lidar_callback(
    table,
    key,
    value,
    isNew
):

    global scan_a_angles
    global scan_a_distances

    global scan_b_angles
    global scan_b_distances


    with data_lock:

        if key == "ScanA":

            (
                scan_a_angles,
                scan_a_distances
            ) = process_scan(
                value
            )


        elif key == "ScanB":

            (
                scan_b_angles,
                scan_b_distances
            ) = process_scan(
                value
            )


# =========================================================
# DISPLAY UPDATE
# =========================================================

def update_display(frame):

    update_robot_pose()


    with data_lock:

        # =================================================
        # COMBINE LIVE SCAN A + B
        # =================================================

        angles = np.array(
            scan_a_angles
            +
            scan_b_angles
        )


        distances = np.array(
            scan_a_distances
            +
            scan_b_distances
        )


        grid_copy = (
            grid.copy()
        )


        current_x = (
            robot_x
        )

        current_y = (
            robot_y
        )

        current_heading = (
            robot_heading_deg
        )


    # =====================================================
    # VIEW 1 - LIVE RADAR
    # =====================================================

    radar_ax.clear()


    radar_ax.set_theta_zero_location(
        "N"
    )


    radar_ax.set_theta_direction(
        -1
    )


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
            distances
            /
            1000.0
        )


        # -------------------------------------------------
        # LASER RAYS
        # -------------------------------------------------

        for angle, distance in zip(
            angles_rad,
            distances_m
        ):

            radar_ax.plot(
                [angle, angle],
                [0, distance],
                linewidth=0.3,
                alpha=0.15
            )


        # -------------------------------------------------
        # CURRENT DETECTION POINTS
        # -------------------------------------------------

        radar_ax.scatter(
            angles_rad,
            distances_m,
            s=10
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


    robot_gx, robot_gy = world_to_grid(
        current_x,
        current_y
    )


    grid_robot_marker.set_data(
        [robot_gx],
        [robot_gy]
    )


    # =====================================================
    # VIEW 3 - LIVE WORLD DETECTIONS ONLY
    # =====================================================

    map_ax.clear()


    map_ax.set_title(
        "3. Live LiDAR Detection"
    )


    map_ax.set_xlabel(
        "X (m)"
    )


    map_ax.set_ylabel(
        "Y (m)"
    )


    # =====================================================
    # KEEP VIEW CENTRED ON ROBOT
    # =====================================================

    map_ax.set_xlim(
        current_x
        -
        MAX_DISTANCE_METERS,

        current_x
        +
        MAX_DISTANCE_METERS
    )


    map_ax.set_ylim(
        current_y
        -
        MAX_DISTANCE_METERS,

        current_y
        +
        MAX_DISTANCE_METERS
    )


    map_ax.set_aspect(
        "equal"
    )


    map_ax.grid(
        True
    )


    # =====================================================
    # CONVERT CURRENT SCAN ONLY TO WORLD COORDINATES
    # =====================================================

    live_x = []
    live_y = []


    for angle, distance in zip(
        angles,
        distances
    ):

        # Only valid current detections

        if (
            distance >= 120
            and
            distance <= 5000
        ):

            hit_x, hit_y = lidar_to_world(
                angle,
                distance
            )


            live_x.append(
                hit_x
            )

            live_y.append(
                hit_y
            )


    # =====================================================
    # SHOW ONLY CURRENT DETECTIONS
    # =====================================================

    if len(live_x) > 0:

        map_ax.scatter(
            live_x,
            live_y,
            s=8
        )


    # =====================================================
    # ROBOT LOCATION
    # =====================================================

    map_ax.scatter(
        [current_x],
        [current_y],
        s=80
    )


    # =====================================================
    # ROBOT HEADING ARROW
    # =====================================================

    heading_rad = np.radians(
        current_heading
    )


    arrow_length = 0.5


    arrow_x = (
        current_x
        +
        arrow_length
        *
        np.sin(
            heading_rad
        )
    )


    arrow_y = (
        current_y
        +
        arrow_length
        *
        np.cos(
            heading_rad
        )
    )


    map_ax.plot(
        [current_x, arrow_x],
        [current_y, arrow_y],
        linewidth=2
    )


    # =====================================================
    # POSE TEXT
    # =====================================================

    map_ax.text(
        current_x
        -
        4.7,

        current_y
        +
        4.3,

        f"X: {current_x:.2f} m\n"
        f"Y: {current_y:.2f} m\n"
        f"Heading: {current_heading:.1f}°\n"
        f"Detected: {len(live_x)}"
    )


    return (
        grid_image,
        grid_robot_marker
    )


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


    time.sleep(
        2
    )


    print(
        "Connected:",
        NetworkTables.isConnected()
    )


    # =====================================================
    # NETWORK TABLES
    # =====================================================

    global lidar_table
    global pose_table


    lidar_table = NetworkTables.getTable(
        "Lidar"
    )


    pose_table = NetworkTables.getTable(
        "RobotPose"
    )


    lidar_table.addEntryListener(
        lidar_callback,
        key="ScanA"
    )


    lidar_table.addEntryListener(
        lidar_callback,
        key="ScanB"
    )


    # =====================================================
    # CREATE WINDOW
    # =====================================================

    global radar_ax
    global grid_ax
    global map_ax

    global grid_image
    global grid_robot_marker


    fig = plt.figure(
        figsize=(18, 6)
    )


    # =====================================================
    # 1. LIVE RADAR
    # =====================================================

    radar_ax = fig.add_subplot(
        1,
        3,
        1,
        projection="polar"
    )


    # =====================================================
    # 2. OCCUPANCY GRID
    # =====================================================

    grid_ax = fig.add_subplot(
        1,
        3,
        2
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


    centre = (
        GRID_SIZE
        //
        2
    )


    grid_robot_marker, = grid_ax.plot(
        [centre],
        [centre],
        "ro"
    )


    # =====================================================
    # 3. LIVE DETECTION MAP
    # =====================================================

    map_ax = fig.add_subplot(
        1,
        3,
        3
    )


    # =====================================================
    # ANIMATION
    # =====================================================

    ani = FuncAnimation(
        fig,
        update_display,
        interval=100,
        cache_frame_data=False
    )


    plt.tight_layout()


    print(
        "Launching LiDAR visualizer..."
    )


    plt.show()


if __name__ == "__main__":
    main()