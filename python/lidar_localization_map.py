import math
import time
import csv
import os

import numpy as np
import matplotlib.pyplot as plt

from networktables import NetworkTables
from scipy.spatial import cKDTree


# ============================================================
# NETWORKTABLES
# ============================================================

ROBOT_IP = "10.23.45.2"

NetworkTables.initialize(
    server=ROBOT_IP
)

lidar_table = NetworkTables.getTable(
    "Lidar"
)

localization_table = NetworkTables.getTable(
    "LidarLocalization"
)


# ============================================================
# SAVE FOLDER
# ============================================================

SCRIPT_FOLDER = os.path.dirname(
    os.path.abspath(__file__)
)

MAP_SAVE_FOLDER = os.path.join(
    SCRIPT_FOLDER,
    "map"
)

os.makedirs(
    MAP_SAVE_FOLDER,
    exist_ok=True
)


# ============================================================
# POSITION LOG FILE
#
# A NEW LOG FILE IS CREATED EACH TIME THE PROGRAM STARTS.
#
# Example:
#
# map/position_log_20260914_121730.csv
# ============================================================

POSITION_LOG_TIMESTAMP = time.strftime(
    "%Y%m%d_%H%M%S"
)

POSITION_LOG_FILE = os.path.join(
    MAP_SAVE_FOLDER,
    "position_log_"
    +
    POSITION_LOG_TIMESTAMP
    +
    ".csv"
)


# ============================================================
# LOGGED POSITIONS
# ============================================================

logged_positions = []

position_counter = 0


# ============================================================
# CREATE POSITION LOG FILE
# ============================================================

def create_position_log_file():

    with open(
        POSITION_LOG_FILE,
        "w",
        newline=""
    ) as file:

        writer = csv.writer(
            file
        )

        writer.writerow(
            [
                "Position",
                "X_m",
                "Y_m",
                "Heading_deg",
                "Timestamp"
            ]
        )


# Create it immediately.
create_position_log_file()


# ============================================================
# LIDAR BASIC SETTINGS
# ============================================================

MIN_DISTANCE_MM = 120.0

MAX_DISTANCE_MM = 5000.0

MIN_SCAN_POINTS = 30


# ============================================================
# LIDAR ANGLE CALIBRATION
# ============================================================

LIDAR_MOUNT_OFFSET_DEG = -20.0


# ============================================================
# BACK WALL DISTANCE CALIBRATION
#
# Robot starting position:
#
# LiDAR centre -> rear wall = exactly 1.60 m
# ============================================================

BACK_WALL_REFERENCE_DISTANCE_M = 1.60

BACK_WALL_SECTOR_START_DEG = 175.0

BACK_WALL_SECTOR_END_DEG = 185.0

BACK_WALL_CALIBRATION_FRAMES = 25

BACK_WALL_MIN_POINTS_PER_FRAME = 5


MIN_ALLOWED_DISTANCE_SCALE = 0.70

MAX_ALLOWED_DISTANCE_SCALE = 1.35


DISTANCE_SCALE_FACTOR = 1.0


distance_calibrated = False


back_wall_calibration_samples = []


back_wall_raw_distance_m = 0.0


back_wall_current_distance_m = 0.0


# ============================================================
# ICP SETTINGS
# ============================================================

MAX_ICP_POINTS = 350

ICP_ITERATIONS = 15

MAX_CORRESPONDENCE_DISTANCE = 0.40

MIN_ICP_MATCHES = 25


# ============================================================
# ICP QUALITY SETTINGS
# ============================================================

MAX_ACCEPTED_ICP_ERROR_M = 0.08

MIN_ACCEPTED_INLIER_RATIO = 0.35

MAX_RELATIVE_TRANSLATION_M = 0.75

MAX_RELATIVE_ROTATION_DEG = 35.0


# ============================================================
# KEYFRAME SETTINGS
# ============================================================

KEYFRAME_TRANSLATION_M = 0.25

KEYFRAME_ROTATION_DEG = 8.0


# ============================================================
# STATIONARY LOCK SETTINGS
# ============================================================

STATIONARY_LOCK_TRANSLATION_M = 0.015

STATIONARY_LOCK_ROTATION_DEG = 0.50


# ============================================================
# LIVE FILTER
# ============================================================

LIVE_CLUSTER_GAP_M = 0.20

LIVE_MIN_CLUSTER_POINTS = 4

MAX_LIVE_LABELS = 8


# ============================================================
# MAP CLUSTER SETTINGS
# ============================================================

MAP_CLUSTER_DISTANCE_M = 0.24

MAP_MIN_CLUSTER_POINTS = 3

MAX_MAP_LABELS = 10


# ============================================================
# DISPLAY POINT SIZES
# ============================================================

LIVE_POINT_SIZE = 12

MAP_POINT_SIZE = 10

SAVED_MAP_POINT_SIZE = 10


# ============================================================
# LOGGED POSITION DISPLAY
# ============================================================

LOGGED_POSITION_POINT_SIZE = 60

LOGGED_POSITION_LABEL_OFFSET_M = 0.12


# ============================================================
# OCCUPANCY MAP
# ============================================================

GRID_SIZE_M = 0.05

OCCUPIED_CONFIRM_THRESHOLD = 3

MAX_CELL_CONFIDENCE = 10

MAP_UPDATE_EVERY_N_FRAMES = 2


# ============================================================
# DISPLAY SETTINGS
# ============================================================

LIVE_HALF_RANGE_M = 5.0

LIVE_RECENTER_THRESHOLD_M = 2.5

MAP_INITIAL_HALF_RANGE_M = 5.0

MAP_EXPAND_MARGIN_M = 1.0

DISPLAY_UPDATE_TIME_S = 0.05


# ============================================================
# ROBOT POSE
# ============================================================

robot_x = 0.0

robot_y = 0.0

robot_heading = 0.0


robot_rotation = np.eye(2)


robot_translation = np.array(
    [0.0, 0.0],
    dtype=float
)


# ============================================================
# KEYFRAME STORAGE
# ============================================================

keyframe_scan = None


keyframe_rotation = np.eye(2)


keyframe_translation = np.array(
    [0.0, 0.0],
    dtype=float
)


keyframe_number = 0


# ============================================================
# LOCALIZATION STATUS
# ============================================================

localization_valid = False

pose_locked = False

frame_counter = 0

last_icp_error = 999.0

last_inlier_ratio = 0.0


# ============================================================
# POSITION LOG STATUS
# ============================================================

last_log_message = "No position logged yet"


# ============================================================
# OCCUPANCY GRID
# ============================================================

occupancy_grid = {}


# ============================================================
# DISPLAY STORAGE
# ============================================================

live_view_center_x = 0.0

live_view_center_y = 0.0


map_min_x = -MAP_INITIAL_HALF_RANGE_M

map_max_x = MAP_INITIAL_HALF_RANGE_M

map_min_y = -MAP_INITIAL_HALF_RANGE_M

map_max_y = MAP_INITIAL_HALF_RANGE_M


# ============================================================
# ANGLE HELPERS
# ============================================================

def normalize_angle(
        angle_deg):

    while angle_deg >= 360.0:

        angle_deg -= 360.0


    while angle_deg < 0.0:

        angle_deg += 360.0


    return angle_deg


def normalize_heading(
        angle_deg):

    while angle_deg > 180.0:

        angle_deg -= 360.0


    while angle_deg < -180.0:

        angle_deg += 360.0


    return angle_deg


def angle_inside_range(
        angle,
        start_angle,
        end_angle):

    if start_angle <= end_angle:

        return (
            angle >= start_angle
            and
            angle <= end_angle
        )


    return (
        angle >= start_angle
        or
        angle <= end_angle
    )


# ============================================================
# PARSE RAW LIDAR
# ============================================================

def parse_scan_raw(
        scan_array):

    angles = []

    raw_distances = []


    if scan_array is None:

        return (
            angles,
            raw_distances
        )


    if len(scan_array) < 2:

        return (
            angles,
            raw_distances
        )


    usable_length = (
        len(scan_array)
        -
        (
            len(scan_array)
            %
            2
        )
    )


    for i in range(
        0,
        usable_length,
        2
    ):

        raw_angle = float(
            scan_array[i]
        )


        raw_distance_mm = float(
            scan_array[i + 1]
        )


        if (
            raw_distance_mm >= MIN_DISTANCE_MM
            and
            raw_distance_mm <= MAX_DISTANCE_MM
        ):

            corrected_angle = normalize_angle(
                raw_angle
                +
                LIDAR_MOUNT_OFFSET_DEG
            )


            angles.append(
                corrected_angle
            )


            raw_distances.append(
                raw_distance_mm
            )


    return (
        angles,
        raw_distances
    )


# ============================================================
# GET FULL RAW SCAN
# ============================================================

def get_full_raw_scan():

    scan_a = lidar_table.getNumberArray(
        "ScanA",
        []
    )


    scan_b = lidar_table.getNumberArray(
        "ScanB",
        []
    )


    angles_a, distances_a = parse_scan_raw(
        scan_a
    )


    angles_b, distances_b = parse_scan_raw(
        scan_b
    )


    angles = (
        angles_a
        +
        angles_b
    )


    raw_distances = (
        distances_a
        +
        distances_b
    )


    if len(angles) == 0:

        return (
            np.array([]),
            np.array([])
        )


    angles = np.array(
        angles,
        dtype=float
    )


    raw_distances = np.array(
        raw_distances,
        dtype=float
    )


    order = np.argsort(
        angles
    )


    return (
        angles[order],
        raw_distances[order]
    )


# ============================================================
# GET REAR SECTOR
# ============================================================

def get_rear_sector_distances(
        angles,
        distances_mm):

    rear_distances = []


    for angle, distance in zip(
        angles,
        distances_mm
    ):

        if angle_inside_range(
            angle,
            BACK_WALL_SECTOR_START_DEG,
            BACK_WALL_SECTOR_END_DEG
        ):

            rear_distances.append(
                float(distance)
            )


    return np.array(
        rear_distances,
        dtype=float
    )


# ============================================================
# DISTANCE CALIBRATION
# ============================================================

def update_distance_calibration(
        angles,
        raw_distances_mm):

    global DISTANCE_SCALE_FACTOR

    global distance_calibrated

    global back_wall_calibration_samples

    global back_wall_raw_distance_m


    if distance_calibrated:

        return


    rear_distances = get_rear_sector_distances(
        angles,
        raw_distances_mm
    )


    if (
        len(rear_distances)
        <
        BACK_WALL_MIN_POINTS_PER_FRAME
    ):

        return


    frame_median_mm = float(
        np.median(
            rear_distances
        )
    )


    frame_median_m = (
        frame_median_mm
        /
        1000.0
    )


    back_wall_calibration_samples.append(
        frame_median_m
    )


    if (
        len(
            back_wall_calibration_samples
        )
        <
        BACK_WALL_CALIBRATION_FRAMES
    ):

        return


    measured_raw_distance_m = float(
        np.median(
            np.array(
                back_wall_calibration_samples,
                dtype=float
            )
        )
    )


    if measured_raw_distance_m <= 0.0:

        back_wall_calibration_samples = []

        return


    candidate_scale = (
        BACK_WALL_REFERENCE_DISTANCE_M
        /
        measured_raw_distance_m
    )


    if (
        candidate_scale
        <
        MIN_ALLOWED_DISTANCE_SCALE
        or
        candidate_scale
        >
        MAX_ALLOWED_DISTANCE_SCALE
    ):

        print(
            "BACK WALL CALIBRATION REJECTED"
        )


        print(
            "Measured:",
            measured_raw_distance_m,
            "m"
        )


        print(
            "Candidate scale:",
            candidate_scale
        )


        back_wall_calibration_samples = []


        return


    back_wall_raw_distance_m = (
        measured_raw_distance_m
    )


    DISTANCE_SCALE_FACTOR = (
        candidate_scale
    )


    distance_calibrated = True


    print(
        "=========================================="
    )

    print(
        "DISTANCE CALIBRATION COMPLETE"
    )

    print(
        "Raw rear wall:",
        back_wall_raw_distance_m,
        "m"
    )

    print(
        "Known rear wall:",
        BACK_WALL_REFERENCE_DISTANCE_M,
        "m"
    )

    print(
        "Distance scale:",
        DISTANCE_SCALE_FACTOR
    )

    print(
        "=========================================="
    )


# ============================================================
# APPLY DISTANCE CALIBRATION
# ============================================================

def apply_distance_calibration(
        raw_distances_mm):

    return (
        raw_distances_mm
        *
        DISTANCE_SCALE_FACTOR
    )


# ============================================================
# CURRENT BACK DISTANCE
# ============================================================

def get_current_back_distance(
        angles,
        corrected_distances_mm):

    rear_distances = get_rear_sector_distances(
        angles,
        corrected_distances_mm
    )


    if len(rear_distances) == 0:

        return 9999.0


    return (
        float(
            np.median(
                rear_distances
            )
        )
        /
        1000.0
    )


# ============================================================
# POLAR -> XY
# ============================================================

def scan_to_xy(
        angles_deg,
        distances_mm):

    if len(angles_deg) == 0:

        return np.empty(
            (0, 2)
        )


    angles_rad = np.radians(
        angles_deg
    )


    distances_m = (
        distances_mm
        /
        1000.0
    )


    local_x = (
        distances_m
        *
        np.sin(
            angles_rad
        )
    )


    local_y = (
        distances_m
        *
        np.cos(
            angles_rad
        )
    )


    return np.column_stack(
        (
            local_x,
            local_y
        )
    )


# ============================================================
# DOWNSAMPLE
# ============================================================

def downsample_points(
        points,
        max_points):

    if len(points) <= max_points:

        return points.copy()


    indices = np.linspace(
        0,
        len(points) - 1,
        max_points,
        dtype=int
    )


    return points[
        indices
    ]


# ============================================================
# BEST FIT TRANSFORM
# ============================================================

def best_fit_transform(
        source,
        target):

    source_center = np.mean(
        source,
        axis=0
    )


    target_center = np.mean(
        target,
        axis=0
    )


    source_zero = (
        source
        -
        source_center
    )


    target_zero = (
        target
        -
        target_center
    )


    H = (
        source_zero.T
        @
        target_zero
    )


    U, _, VT = np.linalg.svd(
        H
    )


    R = (
        VT.T
        @
        U.T
    )


    if np.linalg.det(R) < 0:

        VT[-1, :] *= -1


        R = (
            VT.T
            @
            U.T
        )


    t = (
        target_center
        -
        R
        @
        source_center
    )


    return (
        R,
        t
    )


# ============================================================
# ICP
# ============================================================

def icp_scan_match(
        current_points,
        keyframe_points):

    current = downsample_points(
        current_points,
        MAX_ICP_POINTS
    )


    reference = downsample_points(
        keyframe_points,
        MAX_ICP_POINTS
    )


    if (
        len(current) < MIN_ICP_MATCHES
        or
        len(reference) < MIN_ICP_MATCHES
    ):

        return (
            np.eye(2),
            np.zeros(2),
            False,
            999.0,
            0.0
        )


    transformed = current.copy()


    total_R = np.eye(2)

    total_t = np.zeros(2)


    tree = cKDTree(
        reference
    )


    previous_error = None

    final_inlier_ratio = 0.0


    for _ in range(
        ICP_ITERATIONS
    ):

        distances, indices = tree.query(
            transformed,
            k=1
        )


        valid = (
            distances
            <
            MAX_CORRESPONDENCE_DISTANCE
        )


        valid_count = int(
            np.count_nonzero(
                valid
            )
        )


        if (
            valid_count
            <
            MIN_ICP_MATCHES
        ):

            return (
                np.eye(2),
                np.zeros(2),
                False,
                999.0,
                0.0
            )


        final_inlier_ratio = (
            valid_count
            /
            float(
                len(transformed)
            )
        )


        source_match = transformed[
            valid
        ]


        target_match = reference[
            indices[valid]
        ]


        R_step, t_step = best_fit_transform(
            source_match,
            target_match
        )


        transformed = (
            transformed
            @
            R_step.T
        ) + t_step


        total_t = (
            R_step
            @
            total_t
            +
            t_step
        )


        total_R = (
            R_step
            @
            total_R
        )


        mean_error = float(
            np.mean(
                distances[valid]
            )
        )


        if previous_error is not None:

            improvement = abs(
                previous_error
                -
                mean_error
            )


            if improvement < 0.0001:

                break


        previous_error = (
            mean_error
        )


    if previous_error is None:

        return (
            np.eye(2),
            np.zeros(2),
            False,
            999.0,
            0.0
        )


    return (
        total_R,
        total_t,
        True,
        previous_error,
        final_inlier_ratio
    )


# ============================================================
# HEADING FROM ROTATION
# ============================================================

def get_heading_from_rotation(
        rotation):

    heading = math.degrees(
        math.atan2(
            rotation[0, 1],
            rotation[1, 1]
        )
    )


    return normalize_heading(
        heading
    )


# ============================================================
# ICP QUALITY
# ============================================================

def is_icp_quality_good(
        delta_R,
        delta_t,
        error,
        inlier_ratio):

    relative_distance = float(
        np.linalg.norm(
            delta_t
        )
    )


    relative_heading = abs(
        get_heading_from_rotation(
            delta_R
        )
    )


    if (
        error
        >
        MAX_ACCEPTED_ICP_ERROR_M
    ):

        return False


    if (
        inlier_ratio
        <
        MIN_ACCEPTED_INLIER_RATIO
    ):

        return False


    if (
        relative_distance
        >
        MAX_RELATIVE_TRANSLATION_M
    ):

        return False


    if (
        relative_heading
        >
        MAX_RELATIVE_ROTATION_DEG
    ):

        return False


    return True


# ============================================================
# UPDATE ROBOT POSE
# ============================================================

def update_pose_from_keyframe(
        relative_R,
        relative_t):

    global robot_rotation

    global robot_translation

    global robot_x

    global robot_y

    global robot_heading

    global pose_locked


    relative_distance = float(
        np.linalg.norm(
            relative_t
        )
    )


    relative_heading = abs(
        get_heading_from_rotation(
            relative_R
        )
    )


    if (
        relative_distance
        <
        STATIONARY_LOCK_TRANSLATION_M
        and
        relative_heading
        <
        STATIONARY_LOCK_ROTATION_DEG
    ):

        robot_rotation = (
            keyframe_rotation.copy()
        )


        robot_translation = (
            keyframe_translation.copy()
        )


        pose_locked = True


    else:

        robot_rotation = (
            keyframe_rotation
            @
            relative_R
        )


        robot_translation = (
            keyframe_translation
            +
            keyframe_rotation
            @
            relative_t
        )


        pose_locked = False


    robot_x = float(
        robot_translation[0]
    )


    robot_y = float(
        robot_translation[1]
    )


    robot_heading = get_heading_from_rotation(
        robot_rotation
    )


# ============================================================
# KEYFRAME CHECK
# ============================================================

def should_create_new_keyframe(
        relative_R,
        relative_t):

    relative_distance = float(
        np.linalg.norm(
            relative_t
        )
    )


    relative_heading = abs(
        get_heading_from_rotation(
            relative_R
        )
    )


    return (
        relative_distance
        >=
        KEYFRAME_TRANSLATION_M
        or
        relative_heading
        >=
        KEYFRAME_ROTATION_DEG
    )


# ============================================================
# CREATE KEYFRAME
# ============================================================

def create_new_keyframe(
        current_scan):

    global keyframe_scan

    global keyframe_rotation

    global keyframe_translation

    global keyframe_number


    keyframe_scan = (
        current_scan.copy()
    )


    keyframe_rotation = (
        robot_rotation.copy()
    )


    keyframe_translation = (
        robot_translation.copy()
    )


    keyframe_number += 1


# ============================================================
# LOCAL -> WORLD
# ============================================================

def local_to_world(
        local_points):

    if len(local_points) == 0:

        return np.empty(
            (0, 2)
        )


    heading_rad = math.radians(
        robot_heading
    )


    cos_h = math.cos(
        heading_rad
    )


    sin_h = math.sin(
        heading_rad
    )


    world_x = (
        robot_x
        +
        local_points[:, 0]
        *
        cos_h
        +
        local_points[:, 1]
        *
        sin_h
    )


    world_y = (
        robot_y
        -
        local_points[:, 0]
        *
        sin_h
        +
        local_points[:, 1]
        *
        cos_h
    )


    return np.column_stack(
        (
            world_x,
            world_y
        )
    )


# ============================================================
# LIVE CLUSTERING
# ============================================================

def find_live_clusters(
        local_points):

    clusters = []


    if len(local_points) == 0:

        return clusters


    current_cluster = [
        local_points[0]
    ]


    for i in range(
        1,
        len(local_points)
    ):

        gap = np.linalg.norm(
            local_points[i]
            -
            local_points[i - 1]
        )


        if (
            gap
            <=
            LIVE_CLUSTER_GAP_M
        ):

            current_cluster.append(
                local_points[i]
            )


        else:

            if (
                len(current_cluster)
                >=
                LIVE_MIN_CLUSTER_POINTS
            ):

                clusters.append(
                    np.array(
                        current_cluster
                    )
                )


            current_cluster = [
                local_points[i]
            ]


    if (
        len(current_cluster)
        >=
        LIVE_MIN_CLUSTER_POINTS
    ):

        clusters.append(
            np.array(
                current_cluster
            )
        )


    return clusters


# ============================================================
# CLUSTERS -> POINTS
# ============================================================

def clusters_to_points(
        clusters):

    if len(clusters) == 0:

        return np.empty(
            (0, 2)
        )


    all_points = []


    for cluster in clusters:

        for point in cluster:

            all_points.append(
                point
            )


    return np.array(
        all_points
    )


# ============================================================
# MAP CLUSTERS
# ============================================================

def find_map_clusters(
        points):

    clusters = []


    if len(points) == 0:

        return clusters


    tree = cKDTree(
        points
    )


    visited = np.zeros(
        len(points),
        dtype=bool
    )


    for start_index in range(
        len(points)
    ):

        if visited[
            start_index
        ]:

            continue


        queue = [
            start_index
        ]


        visited[
            start_index
        ] = True


        cluster_indices = []


        while len(queue) > 0:

            current_index = queue.pop()


            cluster_indices.append(
                current_index
            )


            neighbours = tree.query_ball_point(
                points[current_index],
                r=MAP_CLUSTER_DISTANCE_M
            )


            for neighbour in neighbours:

                if not visited[
                    neighbour
                ]:

                    visited[
                        neighbour
                    ] = True


                    queue.append(
                        neighbour
                    )


        if (
            len(cluster_indices)
            >=
            MAP_MIN_CLUSTER_POINTS
        ):

            clusters.append(
                points[
                    cluster_indices
                ]
            )


    clusters.sort(
        key=len,
        reverse=True
    )


    return clusters


# ============================================================
# CLUSTER INFORMATION
# ============================================================

def get_cluster_information(
        world_cluster):

    if len(world_cluster) == 0:

        return (
            0.0,
            0.0,
            9999.0
        )


    centre = np.mean(
        world_cluster,
        axis=0
    )


    dx = (
        world_cluster[:, 0]
        -
        robot_x
    )


    dy = (
        world_cluster[:, 1]
        -
        robot_y
    )


    distances = np.sqrt(
        dx * dx
        +
        dy * dy
    )


    nearest_distance = float(
        np.min(
            distances
        )
    )


    return (
        float(
            centre[0]
        ),
        float(
            centre[1]
        ),
        nearest_distance
    )


# ============================================================
# OCCUPANCY GRID UPDATE
# ============================================================

def update_occupancy_grid(
        world_points):

    global occupancy_grid


    if len(world_points) == 0:

        return


    seen_this_frame = set()


    for point in world_points:

        grid_x = int(
            round(
                point[0]
                /
                GRID_SIZE_M
            )
        )


        grid_y = int(
            round(
                point[1]
                /
                GRID_SIZE_M
            )
        )


        key = (
            grid_x,
            grid_y
        )


        if key in seen_this_frame:

            continue


        seen_this_frame.add(
            key
        )


        confidence = occupancy_grid.get(
            key,
            0
        )


        confidence += 1


        if (
            confidence
            >
            MAX_CELL_CONFIDENCE
        ):

            confidence = (
                MAX_CELL_CONFIDENCE
            )


        occupancy_grid[
            key
        ] = confidence


# ============================================================
# CONFIRMED MAP POINTS
# ============================================================

def get_confirmed_map_points():

    confirmed = []


    for (
        key,
        confidence
    ) in occupancy_grid.items():

        if (
            confidence
            >=
            OCCUPIED_CONFIRM_THRESHOLD
        ):

            confirmed.append(
                [
                    key[0]
                    *
                    GRID_SIZE_M,

                    key[1]
                    *
                    GRID_SIZE_M
                ]
            )


    if len(confirmed) == 0:

        return np.empty(
            (0, 2)
        )


    return np.array(
        confirmed,
        dtype=float
    )


# ============================================================
# NEAREST OBSTACLE
# ============================================================

def get_nearest_obstacle(
        local_points,
        world_points):

    if len(local_points) == 0:

        return (
            0.0,
            0.0,
            9999.0,
            0.0
        )


    distances = np.linalg.norm(
        local_points,
        axis=1
    )


    index = int(
        np.argmin(
            distances
        )
    )


    local_point = local_points[
        index
    ]


    world_point = world_points[
        index
    ]


    distance = float(
        distances[index]
    )


    angle = math.degrees(
        math.atan2(
            local_point[0],
            local_point[1]
        )
    )


    angle = normalize_angle(
        angle
    )


    return (
        float(
            world_point[0]
        ),

        float(
            world_point[1]
        ),

        distance,

        angle
    )


# ============================================================
# LOG CURRENT ROBOT POSITION
#
# L = LOG
#
# Only allows logging when:
#
# distance calibration finished
# localization valid
# pose lock ON
# ============================================================

def log_current_position():

    global position_counter

    global last_log_message


    if not distance_calibrated:

        last_log_message = (
            "LOG FAILED - DISTANCE NOT CALIBRATED"
        )


        print(
            last_log_message
        )


        return


    if not localization_valid:

        last_log_message = (
            "LOG FAILED - LOCALIZATION NOT VALID"
        )


        print(
            last_log_message
        )


        return


    if not pose_locked:

        last_log_message = (
            "LOG FAILED - WAIT FOR POSE LOCK"
        )


        print(
            last_log_message
        )


        return


    # ========================================================
    # CREATE POSITION NAME
    # ========================================================

    position_counter += 1


    position_name = (
        "P"
        +
        str(
            position_counter
        )
    )


    timestamp = time.strftime(
        "%Y-%m-%d %H:%M:%S"
    )


    # ========================================================
    # SAVE POSITION IN MEMORY
    # ========================================================

    position_data = {
        "name": position_name,
        "x": float(robot_x),
        "y": float(robot_y),
        "heading": float(robot_heading),
        "timestamp": timestamp
    }


    logged_positions.append(
        position_data
    )


    # ========================================================
    # APPEND TO CSV
    # ========================================================

    with open(
        POSITION_LOG_FILE,
        "a",
        newline=""
    ) as file:

        writer = csv.writer(
            file
        )


        writer.writerow(
            [
                position_name,
                robot_x,
                robot_y,
                robot_heading,
                timestamp
            ]
        )


    last_log_message = (
        f"{position_name} LOGGED "
        f"X={robot_x:.2f} "
        f"Y={robot_y:.2f} "
        f"H={robot_heading:.1f}°"
    )


    print(
        "=========================================="
    )

    print(
        last_log_message
    )

    print(
        "Saved to:"
    )

    print(
        POSITION_LOG_FILE
    )

    print(
        "=========================================="
    )


# ============================================================
# NETWORKTABLE OUTPUT
# ============================================================

def publish_localization(
        obstacle_x,
        obstacle_y,
        obstacle_distance,
        obstacle_angle,
        valid):

    localization_table.putNumber(
        "RobotX",
        robot_x
    )


    localization_table.putNumber(
        "RobotY",
        robot_y
    )


    localization_table.putNumber(
        "RobotHeading",
        robot_heading
    )


    localization_table.putNumber(
        "ObstacleX",
        obstacle_x
    )


    localization_table.putNumber(
        "ObstacleY",
        obstacle_y
    )


    localization_table.putNumber(
        "ObstacleDistance",
        obstacle_distance
    )


    localization_table.putNumber(
        "ObstacleAngle",
        obstacle_angle
    )


    localization_table.putNumber(
        "BackWallDistance",
        back_wall_current_distance_m
    )


    localization_table.putNumber(
        "DistanceScale",
        DISTANCE_SCALE_FACTOR
    )


    localization_table.putNumber(
        "LoggedPositionCount",
        len(
            logged_positions
        )
    )


    localization_table.putBoolean(
        "LocalizationValid",
        valid
    )


# ============================================================
# SAVE MAP
# ============================================================

def save_map():

    confirmed_points = (
        get_confirmed_map_points()
    )


    if len(confirmed_points) == 0:

        print(
            "No confirmed map points to save."
        )

        return


    confirmed_clusters = find_map_clusters(
        confirmed_points
    )


    timestamp = time.strftime(
        "%Y%m%d_%H%M%S"
    )


    csv_filename = (
        "lidar_map_"
        +
        timestamp
        +
        ".csv"
    )


    png_filename = (
        "lidar_map_"
        +
        timestamp
        +
        ".png"
    )


    csv_path = os.path.join(
        MAP_SAVE_FOLDER,
        csv_filename
    )


    png_path = os.path.join(
        MAP_SAVE_FOLDER,
        png_filename
    )


    # ========================================================
    # MAP CSV
    # ========================================================

    with open(
        csv_path,
        "w",
        newline=""
    ) as file:

        writer = csv.writer(
            file
        )


        writer.writerow(
            [
                "X_m",
                "Y_m"
            ]
        )


        for point in confirmed_points:

            writer.writerow(
                [
                    point[0],
                    point[1]
                ]
            )


    # ========================================================
    # MAP PNG
    # ========================================================

    save_fig, save_ax = plt.subplots(
        figsize=(9, 9)
    )


    save_ax.set_facecolor(
        "white"
    )


    save_ax.scatter(
        confirmed_points[:, 0],
        confirmed_points[:, 1],
        s=SAVED_MAP_POINT_SIZE,
        c="black",
        marker="s"
    )


    # ========================================================
    # OBJECT LABELS
    # ========================================================

    for cluster in confirmed_clusters[
        :MAX_MAP_LABELS
    ]:

        (
            centre_x,
            centre_y,
            nearest_distance
        ) = get_cluster_information(
            cluster
        )


        save_ax.text(
            centre_x,
            centre_y,
            (
                f"X {centre_x:.2f} "
                f"Y {centre_y:.2f}\n"
                f"D {nearest_distance:.2f} m"
            ),
            fontsize=8,
            fontweight="bold",
            ha="center",
            va="bottom",
            bbox=dict(
                boxstyle="round,pad=0.25",
                facecolor="white",
                edgecolor="black",
                alpha=0.92
            )
        )


    # ========================================================
    # LOGGED POSITIONS ON SAVED MAP
    # ========================================================

    for position in logged_positions:

        save_ax.scatter(
            position["x"],
            position["y"],
            s=LOGGED_POSITION_POINT_SIZE,
            marker="o",
            zorder=10
        )


        save_ax.text(
            position["x"],
            position["y"]
            +
            LOGGED_POSITION_LABEL_OFFSET_M,
            (
                f'{position["name"]}\n'
                f'({position["x"]:.2f}, '
                f'{position["y"]:.2f})\n'
                f'{position["heading"]:.1f}°'
            ),
            fontsize=9,
            fontweight="bold",
            ha="center",
            va="bottom",
            zorder=11,
            bbox=dict(
                boxstyle="round,pad=0.25",
                facecolor="white",
                edgecolor="black",
                alpha=0.95
            )
        )


    # ========================================================
    # CURRENT ROBOT
    # ========================================================

    save_ax.scatter(
        robot_x,
        robot_y,
        s=100,
        marker="^"
    )


    heading_rad = math.radians(
        robot_heading
    )


    heading_dx = (
        0.40
        *
        math.sin(
            heading_rad
        )
    )


    heading_dy = (
        0.40
        *
        math.cos(
            heading_rad
        )
    )


    save_ax.plot(
        [
            robot_x,
            robot_x + heading_dx
        ],
        [
            robot_y,
            robot_y + heading_dy
        ],
        linewidth=2
    )


    save_ax.set_aspect(
        "equal",
        adjustable="box"
    )


    save_ax.set_xlabel(
        "World X (m)"
    )


    save_ax.set_ylabel(
        "World Y (m)"
    )


    save_ax.set_title(
        "LiDAR Confirmed Map"
    )


    save_ax.grid(
        False
    )


    save_fig.savefig(
        png_path,
        dpi=200,
        bbox_inches="tight",
        facecolor="white"
    )


    plt.close(
        save_fig
    )


    print(
        "=========================================="
    )

    print(
        "MAP SAVED"
    )

    print(
        csv_path
    )

    print(
        png_path
    )

    print(
        "=========================================="
    )


# ============================================================
# RESET LOCALIZATION
#
# IMPORTANT:
#
# Logged positions are NOT deleted from the CSV.
#
# However their coordinate frame belongs to the old map.
# Therefore don't use R during a position logging test.
# ============================================================

def reset_localization():

    global robot_x

    global robot_y

    global robot_heading

    global robot_rotation

    global robot_translation

    global keyframe_scan

    global keyframe_rotation

    global keyframe_translation

    global keyframe_number

    global localization_valid

    global pose_locked

    global frame_counter

    global last_icp_error

    global last_inlier_ratio

    global occupancy_grid

    global live_view_center_x

    global live_view_center_y

    global map_min_x

    global map_max_x

    global map_min_y

    global map_max_y


    robot_x = 0.0

    robot_y = 0.0

    robot_heading = 0.0


    robot_rotation = np.eye(2)


    robot_translation = np.array(
        [0.0, 0.0],
        dtype=float
    )


    keyframe_scan = None


    keyframe_rotation = np.eye(2)


    keyframe_translation = np.array(
        [0.0, 0.0],
        dtype=float
    )


    keyframe_number = 0


    localization_valid = False

    pose_locked = False


    frame_counter = 0


    last_icp_error = 999.0

    last_inlier_ratio = 0.0


    occupancy_grid = {}


    live_view_center_x = 0.0

    live_view_center_y = 0.0


    map_min_x = (
        -MAP_INITIAL_HALF_RANGE_M
    )


    map_max_x = (
        MAP_INITIAL_HALF_RANGE_M
    )


    map_min_y = (
        -MAP_INITIAL_HALF_RANGE_M
    )


    map_max_y = (
        MAP_INITIAL_HALF_RANGE_M
    )


    ax_live.set_xlim(
        -LIVE_HALF_RANGE_M,
        LIVE_HALF_RANGE_M
    )


    ax_live.set_ylim(
        -LIVE_HALF_RANGE_M,
        LIVE_HALF_RANGE_M
    )


    ax_map.set_xlim(
        map_min_x,
        map_max_x
    )


    ax_map.set_ylim(
        map_min_y,
        map_max_y
    )


    print(
        "Localization/map reset."
    )


# ============================================================
# KEYBOARD
# ============================================================

def on_key(event):

    if event.key == "s":

        save_map()


    elif event.key == "r":

        reset_localization()


    elif event.key == "l":

        log_current_position()


# ============================================================
# WINDOW
# ============================================================

plt.ion()


fig, (
    ax_live,
    ax_map
) = plt.subplots(
    1,
    2,
    figsize=(16, 8)
)


fig.canvas.mpl_connect(
    "key_press_event",
    on_key
)


fig.suptitle(
    "LiDAR Localization and Mapping",
    fontsize=16
)


plt.show(
    block=False
)


# ============================================================
# ALT+TAB
# ============================================================

try:

    window = fig.canvas.manager.window


    if hasattr(
        window,
        "attributes"
    ):

        window.attributes(
            "-topmost",
            False
        )


except Exception:

    pass


# ============================================================
# LIVE GRAPH
# ============================================================

ax_live.set_facecolor(
    "white"
)


ax_live.set_title(
    "LIVE - Current LiDAR Detections"
)


ax_live.set_xlabel(
    "World X (m)"
)


ax_live.set_ylabel(
    "World Y (m)"
)


ax_live.set_aspect(
    "equal",
    adjustable="box"
)


ax_live.grid(
    False
)


ax_live.set_xlim(
    -LIVE_HALF_RANGE_M,
    LIVE_HALF_RANGE_M
)


ax_live.set_ylim(
    -LIVE_HALF_RANGE_M,
    LIVE_HALF_RANGE_M
)


live_scatter = ax_live.scatter(
    [],
    [],
    s=LIVE_POINT_SIZE,
    c="black",
    marker="s"
)


live_robot, = ax_live.plot(
    [],
    [],
    marker="^",
    linestyle="None",
    markersize=10
)


live_heading_line, = ax_live.plot(
    [],
    [],
    linewidth=2
)


live_hud = ax_live.text(
    0.02,
    0.98,
    "",
    transform=ax_live.transAxes,
    verticalalignment="top",
    fontsize=9,
    bbox=dict(
        boxstyle="round",
        facecolor="white",
        edgecolor="black",
        alpha=0.95
    )
)


# ============================================================
# LIVE LABELS
# ============================================================

live_labels = []


for _ in range(
    MAX_LIVE_LABELS
):

    label = ax_live.text(
        0.0,
        0.0,
        "",
        fontsize=8,
        fontweight="bold",
        ha="center",
        va="bottom",
        visible=False,
        bbox=dict(
            boxstyle="round,pad=0.25",
            facecolor="white",
            edgecolor="black",
            alpha=0.93
        )
    )


    live_labels.append(
        label
    )


# ============================================================
# MAP GRAPH
# ============================================================

ax_map.set_facecolor(
    "white"
)


ax_map.set_title(
    "PERSISTENT - Confirmed Map"
)


ax_map.set_xlabel(
    "World X (m)"
)


ax_map.set_ylabel(
    "World Y (m)"
)


ax_map.set_aspect(
    "equal",
    adjustable="box"
)


ax_map.grid(
    False
)


ax_map.set_xlim(
    map_min_x,
    map_max_x
)


ax_map.set_ylim(
    map_min_y,
    map_max_y
)


map_scatter = ax_map.scatter(
    [],
    [],
    s=MAP_POINT_SIZE,
    c="black",
    marker="s"
)


map_robot, = ax_map.plot(
    [],
    [],
    marker="^",
    linestyle="None",
    markersize=10
)


map_heading_line, = ax_map.plot(
    [],
    [],
    linewidth=2
)


map_robot_label = ax_map.text(
    0.0,
    0.0,
    "",
    fontsize=8,
    fontweight="bold",
    ha="center",
    va="top",
    bbox=dict(
        boxstyle="round,pad=0.20",
        facecolor="white",
        edgecolor="black",
        alpha=0.92
    )
)


# ============================================================
# MAP OBJECT LABELS
# ============================================================

map_labels = []


for _ in range(
    MAX_MAP_LABELS
):

    label = ax_map.text(
        0.0,
        0.0,
        "",
        fontsize=8,
        fontweight="bold",
        ha="center",
        va="bottom",
        visible=False,
        bbox=dict(
            boxstyle="round,pad=0.25",
            facecolor="white",
            edgecolor="black",
            alpha=0.93
        )
    )


    map_labels.append(
        label
    )


# ============================================================
# LOGGED POSITION ARTISTS
# ============================================================

logged_position_scatter = ax_map.scatter(
    [],
    [],
    s=LOGGED_POSITION_POINT_SIZE,
    marker="o",
    zorder=10
)


logged_position_labels = []


# ============================================================
# MAP HUD
# ============================================================

map_hud = ax_map.text(
    0.02,
    0.98,
    "",
    transform=ax_map.transAxes,
    verticalalignment="top",
    fontsize=9,
    bbox=dict(
        boxstyle="round",
        facecolor="white",
        edgecolor="black",
        alpha=0.95
    )
)


fig.tight_layout(
    rect=[
        0.0,
        0.0,
        1.0,
        0.95
    ]
)


# ============================================================
# START
# ============================================================

print(
    "=========================================="
)

print(
    " LIDAR LOCALIZATION + POSITION LOGGING"
)

print(
    "=========================================="
)

print(
    "L = Log current position"
)

print(
    "S = Save map"
)

print(
    "R = Reset localization/map"
)

print(
    ""
)

print(
    "Position log:"
)

print(
    POSITION_LOG_FILE
)

print(
    ""
)

print(
    "Robot must start 1.60 m from rear wall."
)

print(
    "Keep robot stationary while calibrating."
)


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while plt.fignum_exists(
        fig.number
    ):

        # ====================================================
        # RAW SCAN
        # ====================================================

        (
            angles,
            raw_distances
        ) = get_full_raw_scan()


        if (
            len(angles)
            <
            MIN_SCAN_POINTS
        ):

            fig.canvas.flush_events()


            time.sleep(
                DISPLAY_UPDATE_TIME_S
            )


            continue


        # ====================================================
        # DISTANCE CALIBRATION
        # ====================================================

        if not distance_calibrated:

            update_distance_calibration(
                angles,
                raw_distances
            )


            raw_points = scan_to_xy(
                angles,
                raw_distances
            )


            live_scatter.set_offsets(
                raw_points
            )


            progress = len(
                back_wall_calibration_samples
            )


            live_hud.set_text(
                (
                    "DISTANCE CALIBRATION\n"
                    "\n"
                    "Keep robot stationary\n"
                    "\n"
                    "Known rear wall: 1.60 m\n"
                    f"Samples: "
                    f"{progress}/"
                    f"{BACK_WALL_CALIBRATION_FRAMES}"
                )
            )


            map_hud.set_text(
                (
                    "WAITING FOR CALIBRATION\n"
                    "\n"
                    f"LiDAR angle: "
                    f"{LIDAR_MOUNT_OFFSET_DEG:.1f}°\n"
                    "Rear reference: 1.60 m\n"
                    "\n"
                    "L = Log position\n"
                    "S = Save map\n"
                    "R = Reset"
                )
            )


            fig.canvas.draw_idle()

            fig.canvas.flush_events()


            time.sleep(
                DISPLAY_UPDATE_TIME_S
            )


            continue


        # ====================================================
        # APPLY DISTANCE SCALE
        # ====================================================

        distances = apply_distance_calibration(
            raw_distances
        )


        # ====================================================
        # CURRENT BACK DISTANCE
        # ====================================================

        back_wall_current_distance_m = (
            get_current_back_distance(
                angles,
                distances
            )
        )


        # ====================================================
        # POINT CLOUD
        # ====================================================

        raw_local_points = scan_to_xy(
            angles,
            distances
        )


        # ====================================================
        # FIRST KEYFRAME
        # ====================================================

        if keyframe_scan is None:

            keyframe_scan = (
                raw_local_points.copy()
            )


            keyframe_rotation = (
                robot_rotation.copy()
            )


            keyframe_translation = (
                robot_translation.copy()
            )


            keyframe_number = 1


            localization_valid = False

            pose_locked = True


            last_icp_error = 999.0

            last_inlier_ratio = 0.0


        else:

            # =================================================
            # CURRENT SCAN -> KEYFRAME
            # =================================================

            (
                relative_R,
                relative_t,
                success,
                icp_error,
                inlier_ratio
            ) = icp_scan_match(
                raw_local_points,
                keyframe_scan
            )


            last_icp_error = (
                icp_error
            )


            last_inlier_ratio = (
                inlier_ratio
            )


            if (
                success
                and
                is_icp_quality_good(
                    relative_R,
                    relative_t,
                    icp_error,
                    inlier_ratio
                )
            ):

                update_pose_from_keyframe(
                    relative_R,
                    relative_t
                )


                localization_valid = True


                if should_create_new_keyframe(
                    relative_R,
                    relative_t
                ):

                    create_new_keyframe(
                        raw_local_points
                    )


            else:

                localization_valid = False

                pose_locked = True


        # ====================================================
        # LIVE FILTER
        # ====================================================

        live_clusters = find_live_clusters(
            raw_local_points
        )


        filtered_local_points = clusters_to_points(
            live_clusters
        )


        # ====================================================
        # WORLD POINTS
        # ====================================================

        filtered_world_points = local_to_world(
            filtered_local_points
        )


        # ====================================================
        # UPDATE MAP
        # ====================================================

        frame_counter += 1


        if (
            localization_valid
            and
            frame_counter
            %
            MAP_UPDATE_EVERY_N_FRAMES
            ==
            0
        ):

            update_occupancy_grid(
                filtered_world_points
            )


        confirmed_map_points = (
            get_confirmed_map_points()
        )


        confirmed_map_clusters = (
            find_map_clusters(
                confirmed_map_points
            )
        )


        # ====================================================
        # NEAREST DETECTION
        # ====================================================

        (
            obstacle_x,
            obstacle_y,
            obstacle_distance,
            obstacle_angle
        ) = get_nearest_obstacle(
            filtered_local_points,
            filtered_world_points
        )


        # ====================================================
        # NETWORKTABLES
        # ====================================================

        publish_localization(
            obstacle_x,
            obstacle_y,
            obstacle_distance,
            obstacle_angle,
            localization_valid
        )


        # ====================================================
        # HEADING ARROW
        # ====================================================

        heading_rad = math.radians(
            robot_heading
        )


        heading_dx = (
            0.40
            *
            math.sin(
                heading_rad
            )
        )


        heading_dy = (
            0.40
            *
            math.cos(
                heading_rad
            )
        )


        # ====================================================
        # LIVE POINTS
        # ====================================================

        if len(
            filtered_world_points
        ) > 0:

            live_scatter.set_offsets(
                filtered_world_points
            )


        else:

            live_scatter.set_offsets(
                np.empty(
                    (0, 2)
                )
            )


        # ====================================================
        # ROBOT
        # ====================================================

        live_robot.set_data(
            [robot_x],
            [robot_y]
        )


        live_heading_line.set_data(
            [
                robot_x,
                robot_x + heading_dx
            ],
            [
                robot_y,
                robot_y + heading_dy
            ]
        )


        # ====================================================
        # LIVE OBJECT LABELS
        # ====================================================

        for label in live_labels:

            label.set_visible(
                False
            )


        for index, cluster in enumerate(
            live_clusters[
                :MAX_LIVE_LABELS
            ]
        ):

            world_cluster = local_to_world(
                cluster
            )


            (
                centre_x,
                centre_y,
                nearest_distance
            ) = get_cluster_information(
                world_cluster
            )


            label = live_labels[
                index
            ]


            label.set_position(
                (
                    centre_x,
                    centre_y
                )
            )


            label.set_text(
                (
                    f"X {centre_x:.2f} "
                    f"Y {centre_y:.2f}\n"
                    f"D {nearest_distance:.2f} m"
                )
            )


            label.set_visible(
                True
            )


        # ====================================================
        # LIVE HUD
        # ====================================================

        live_hud.set_text(
            (
                "ROBOT\n"

                f"X: {robot_x:.2f} m\n"

                f"Y: {robot_y:.2f} m\n"

                f"Heading: "
                f"{robot_heading:.2f}°\n"

                "\n"

                f"ICP Error: "
                f"{last_icp_error:.3f} m\n"

                f"ICP Inliers: "
                f"{last_inlier_ratio * 100:.0f}%\n"

                f"Keyframe: "
                f"{keyframe_number}\n"

                f"Pose lock: "
                f"{'ON' if pose_locked else 'OFF'}\n"

                "\n"

                f"Logged positions: "
                f"{len(logged_positions)}\n"

                f"{last_log_message}\n"

                "\n"

                "NEAREST DETECTION\n"

                f"Distance: "
                f"{obstacle_distance:.2f} m\n"

                f"Angle: "
                f"{obstacle_angle:.1f}°\n"

                "\n"

                f"Localization: "
                f"{'VALID' if localization_valid else 'HOLD'}"
            )
        )


        # ====================================================
        # LIVE RECENTER
        # ====================================================

        if (
            abs(
                robot_x
                -
                live_view_center_x
            )
            >
            LIVE_RECENTER_THRESHOLD_M
        ):

            live_view_center_x = (
                robot_x
            )


            ax_live.set_xlim(
                live_view_center_x
                -
                LIVE_HALF_RANGE_M,

                live_view_center_x
                +
                LIVE_HALF_RANGE_M
            )


        if (
            abs(
                robot_y
                -
                live_view_center_y
            )
            >
            LIVE_RECENTER_THRESHOLD_M
        ):

            live_view_center_y = (
                robot_y
            )


            ax_live.set_ylim(
                live_view_center_y
                -
                LIVE_HALF_RANGE_M,

                live_view_center_y
                +
                LIVE_HALF_RANGE_M
            )


        # ====================================================
        # CONFIRMED MAP
        # ====================================================

        if len(
            confirmed_map_points
        ) > 0:

            map_scatter.set_offsets(
                confirmed_map_points
            )


        else:

            map_scatter.set_offsets(
                np.empty(
                    (0, 2)
                )
            )


        # ====================================================
        # MAP OBJECT LABELS
        # ====================================================

        for label in map_labels:

            label.set_visible(
                False
            )


        for index, cluster in enumerate(
            confirmed_map_clusters[
                :MAX_MAP_LABELS
            ]
        ):

            (
                centre_x,
                centre_y,
                nearest_distance
            ) = get_cluster_information(
                cluster
            )


            label = map_labels[
                index
            ]


            label.set_position(
                (
                    centre_x,
                    centre_y
                )
            )


            label.set_text(
                (
                    f"X {centre_x:.2f} "
                    f"Y {centre_y:.2f}\n"
                    f"D {nearest_distance:.2f} m"
                )
            )


            label.set_visible(
                True
            )


        # ====================================================
        # LOGGED POSITION POINTS
        # ====================================================

        if len(
            logged_positions
        ) > 0:

            logged_xy = np.array(
                [
                    [
                        position["x"],
                        position["y"]
                    ]
                    for position in logged_positions
                ],
                dtype=float
            )


            logged_position_scatter.set_offsets(
                logged_xy
            )


        else:

            logged_position_scatter.set_offsets(
                np.empty(
                    (0, 2)
                )
            )


        # ====================================================
        # DELETE OLD LOGGED POSITION TEXT ARTISTS
        # ====================================================

        for artist in logged_position_labels:

            artist.remove()


        logged_position_labels.clear()


        # ====================================================
        # CREATE LOGGED POSITION LABELS
        # ====================================================

        for position in logged_positions:

            artist = ax_map.text(
                position["x"],
                position["y"]
                +
                LOGGED_POSITION_LABEL_OFFSET_M,
                (
                    f'{position["name"]}\n'
                    f'({position["x"]:.2f}, '
                    f'{position["y"]:.2f})\n'
                    f'{position["heading"]:.1f}°'
                ),
                fontsize=9,
                fontweight="bold",
                ha="center",
                va="bottom",
                zorder=11,
                bbox=dict(
                    boxstyle="round,pad=0.25",
                    facecolor="white",
                    edgecolor="black",
                    alpha=0.95
                )
            )


            logged_position_labels.append(
                artist
            )


        # ====================================================
        # MAP ROBOT
        # ====================================================

        map_robot.set_data(
            [robot_x],
            [robot_y]
        )


        map_heading_line.set_data(
            [
                robot_x,
                robot_x + heading_dx
            ],
            [
                robot_y,
                robot_y + heading_dy
            ]
        )


        map_robot_label.set_position(
            (
                robot_x,
                robot_y - 0.20
            )
        )


        map_robot_label.set_text(
            (
                "ROBOT\n"
                f"X {robot_x:.2f}\n"
                f"Y {robot_y:.2f}\n"
                f"{robot_heading:.1f}°"
            )
        )


        # ====================================================
        # MAP HUD
        # ====================================================

        map_hud.set_text(
            (
                f"Robot X: "
                f"{robot_x:.2f} m\n"

                f"Robot Y: "
                f"{robot_y:.2f} m\n"

                f"Heading: "
                f"{robot_heading:.2f}°\n"

                f"Keyframe: "
                f"{keyframe_number}\n"

                f"Pose lock: "
                f"{'ON' if pose_locked else 'OFF'}\n"

                "\n"

                f"Logged positions: "
                f"{len(logged_positions)}\n"

                "\n"

                f"Confirmed cells: "
                f"{len(confirmed_map_points)}\n"

                f"Confirmed objects: "
                f"{len(confirmed_map_clusters)}\n"

                "\n"

                f"Distance scale: "
                f"{DISTANCE_SCALE_FACTOR:.4f}\n"

                f"Current rear: "
                f"{back_wall_current_distance_m:.2f} m\n"

                "\n"

                "L = Log position\n"

                "S = Save map\n"

                "R = Reset map"
            )
        )


        # ====================================================
        # MAP RANGE
        # ====================================================

        if len(
            confirmed_map_points
        ) > 0:

            required_min_x = min(
                float(
                    np.min(
                        confirmed_map_points[:, 0]
                    )
                ),
                robot_x
            )


            required_max_x = max(
                float(
                    np.max(
                        confirmed_map_points[:, 0]
                    )
                ),
                robot_x
            )


            required_min_y = min(
                float(
                    np.min(
                        confirmed_map_points[:, 1]
                    )
                ),
                robot_y
            )


            required_max_y = max(
                float(
                    np.max(
                        confirmed_map_points[:, 1]
                    )
                ),
                robot_y
            )


            limits_changed = False


            if (
                required_min_x
                <
                map_min_x
                +
                MAP_EXPAND_MARGIN_M
            ):

                map_min_x = (
                    required_min_x
                    -
                    MAP_EXPAND_MARGIN_M
                )


                limits_changed = True


            if (
                required_max_x
                >
                map_max_x
                -
                MAP_EXPAND_MARGIN_M
            ):

                map_max_x = (
                    required_max_x
                    +
                    MAP_EXPAND_MARGIN_M
                )


                limits_changed = True


            if (
                required_min_y
                <
                map_min_y
                +
                MAP_EXPAND_MARGIN_M
            ):

                map_min_y = (
                    required_min_y
                    -
                    MAP_EXPAND_MARGIN_M
                )


                limits_changed = True


            if (
                required_max_y
                >
                map_max_y
                -
                MAP_EXPAND_MARGIN_M
            ):

                map_max_y = (
                    required_max_y
                    +
                    MAP_EXPAND_MARGIN_M
                )


                limits_changed = True


            if limits_changed:

                ax_map.set_xlim(
                    map_min_x,
                    map_max_x
                )


                ax_map.set_ylim(
                    map_min_y,
                    map_max_y
                )


        # ====================================================
        # DISPLAY
        # ====================================================

        fig.canvas.draw_idle()


        fig.canvas.flush_events()


        time.sleep(
            DISPLAY_UPDATE_TIME_S
        )


# ============================================================
# STOP
# ============================================================

except KeyboardInterrupt:

    print(
        "\nLiDAR localization stopped."
    )


finally:

    localization_table.putBoolean(
        "LocalizationValid",
        False
    )


    print(
        "Position log saved at:"
    )


    print(
        POSITION_LOG_FILE
    )


    print(
        "Localization closed."
    )