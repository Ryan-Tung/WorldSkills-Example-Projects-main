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

robot_pose_table = NetworkTables.getTable(
    "RobotPose"
)

keyboard_table = NetworkTables.getTable(
    "KeyboardDrive"
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
# POSITION LOG
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

logged_positions = []

position_counter = 0

last_log_message = "No position logged yet"


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


create_position_log_file()


# ============================================================
# LIDAR SETTINGS
# ============================================================

MIN_DISTANCE_MM = 120.0

MAX_DISTANCE_MM = 5000.0

MIN_SCAN_POINTS = 30


# ============================================================
# LIDAR MOUNTING ANGLE
# ============================================================

LIDAR_MOUNT_OFFSET_DEG = -20.0


# ============================================================
# DISTANCE SCALE
#
# Flexible arena.
# No known wall is required.
# ============================================================

DISTANCE_SCALE_FACTOR = 1.0


# ============================================================
# NAVX
#
# navX is now responsible for heading.
#
# If physical left/right direction is reversed,
# change 1.0 to -1.0.
# ============================================================

NAVX_HEADING_SIGN = 1.0


# ============================================================
# NAVX ZERO
#
# Python keeps its own zero reference.
#
# Press R:
#
# current robot direction = 0°
# ============================================================

navx_zero_offset = 0.0

navx_zero_initialized = False

navx_available = False

raw_navx_heading = 0.0


# ============================================================
# ROBOT HEADING
# ============================================================

robot_heading = 0.0

previous_navx_heading = 0.0

keyframe_navx_heading = 0.0


# ============================================================
# NORMAL TRANSLATION ICP
#
# navX gives rotation.
#
# ICP is only used to estimate translation.
# ============================================================

NORMAL_ICP_POINTS = 260

NORMAL_ICP_ITERATIONS = 8

NORMAL_CORRESPONDENCE_M = 0.45

NORMAL_MIN_MATCHES = 20

NORMAL_MAX_ERROR_M = 0.10

NORMAL_MIN_INLIER_RATIO = 0.30

NORMAL_MAX_TRANSLATION_M = 0.45


# ============================================================
# RECOVERY ICP
# ============================================================

RECOVERY_ICP_POINTS = 320

RECOVERY_ICP_ITERATIONS = 12

RECOVERY_CORRESPONDENCE_M = 0.70

RECOVERY_MIN_MATCHES = 15

RECOVERY_MAX_ERROR_M = 0.16

RECOVERY_MIN_INLIER_RATIO = 0.20

RECOVERY_MAX_TRANSLATION_M = 1.00


# ============================================================
# PURE ROTATION COMMAND DETECTION
#
# Q / E:
#
# X ≈ 0
# Y ≈ 0
# Z != 0
# ============================================================

KEYBOARD_MOVE_THRESHOLD = 0.05

KEYBOARD_TURN_THRESHOLD = 0.05


# ============================================================
# BACKUP ROTATION TRANSLATION SUPPRESSION
# ============================================================

ROTATION_SUPPRESSION_MIN_DEG = 0.40

ROTATION_SUPPRESSION_MAX_TRANSLATION_M = 0.10


# ============================================================
# KEYFRAME
# ============================================================

KEYFRAME_TRANSLATION_M = 0.20

KEYFRAME_ROTATION_DEG = 8.0


# ============================================================
# STATIONARY TRANSLATION FILTER
# ============================================================

STATIONARY_TRANSLATION_M = 0.006


# ============================================================
# LIVE POINT FILTER
# ============================================================

LIVE_CLUSTER_GAP_M = 0.20

LIVE_MIN_CLUSTER_POINTS = 4


# ============================================================
# OCCUPANCY GRID
#
# Fixed Cartographer-style map:
#
# - 7 m x 7 m
# - 5 cm per cell
# - unknown starts grey
# - LiDAR ray path adds FREE evidence
# - LiDAR endpoint adds OCCUPIED evidence
# - repeated free observations can remove old false obstacles
# ============================================================

MAP_SIZE_M = 7.0

GRID_SIZE_M = 0.05

MAP_HALF_SIZE_M = (
    MAP_SIZE_M
    /
    2.0
)

MAP_CELL_COUNT = int(
    round(
        MAP_SIZE_M
        /
        GRID_SIZE_M
    )
)

FREE_CELL_UPDATE = -1

OCCUPIED_CELL_UPDATE = 2

FREE_CELL_THRESHOLD = -1

OCCUPIED_CONFIRM_THRESHOLD = 4

MIN_CELL_CONFIDENCE = -12

MAX_CELL_CONFIDENCE = 12

MAP_UPDATE_EVERY_N_FRAMES = 2

# Use every second filtered LiDAR point for ray tracing.
# This keeps the map responsive without losing wall detail.
MAP_RAY_STRIDE = 2


# ============================================================
# DISPLAY
# ============================================================

LIVE_POINT_SIZE = 10

MAP_POINT_SIZE = 9

SAVED_MAP_POINT_SIZE = 9

LIVE_HALF_RANGE_M = 5.0

LIVE_RECENTER_THRESHOLD_M = 2.5

MAP_INITIAL_HALF_RANGE_M = MAP_HALF_SIZE_M

# Persistent occupancy map is fixed-size and does not auto-expand.
MAP_EXPAND_MARGIN_M = 0.0

DISPLAY_UPDATE_TIME_S = 0.02


# ============================================================
# ROBOT POSITION
#
# X/Y = LiDAR translation
# Heading = navX
# ============================================================

robot_x = 0.0

robot_y = 0.0

robot_translation = np.array(
    [
        0.0,
        0.0
    ],
    dtype=float
)


# ============================================================
# PREVIOUS GOOD SCAN
# ============================================================

previous_good_scan = None

previous_good_translation = np.array(
    [
        0.0,
        0.0
    ],
    dtype=float
)


# ============================================================
# KEYFRAME
# ============================================================

keyframe_scan = None

keyframe_translation = np.array(
    [
        0.0,
        0.0
    ],
    dtype=float
)

keyframe_number = 0


# ============================================================
# STATUS
# ============================================================

localization_valid = False

localization_state = "STARTING"

pose_locked = False

recovery_count = 0

rotation_hold_count = 0

consecutive_hold_count = 0

frame_counter = 0

last_icp_error = 999.0

last_inlier_ratio = 0.0

last_relative_translation = 0.0

last_navx_delta = 0.0

# Monotonic counters for monitors / fusion / debugging.
localization_sample_id = 0
localization_reset_id = 0


# ============================================================
# OCCUPANCY GRID STORAGE
#
# rows = map Y
# cols = map X
#
#  0 = unknown
# <0 = free-space evidence
# >0 = obstacle evidence
# ============================================================

occupancy_grid = np.zeros(
    (
        MAP_CELL_COUNT,
        MAP_CELL_COUNT
    ),
    dtype=np.int16
)


# ============================================================
# MAPPING POSE
#
# The persistent map uses the SAME localization pose produced here:
#
# LiDAR ICP X/Y + navX heading
#
# No encoder X/Y is used for the persistent map.
#
# Display convention matches keyboard tracking:
#
# 0 degrees   = +X = right
# 90 degrees  = -Y = down
# 180 degrees = -X = left
# ============================================================

map_pose_zero_initialized = False

map_pose_zero_x = 0.0

map_pose_zero_y = 0.0

map_pose_zero_heading = 0.0

mapping_robot_x = 0.0

mapping_robot_y = 0.0

mapping_robot_heading = 0.0

mapping_pose_available = False


# ============================================================
# DISPLAY LIMITS
# ============================================================

live_view_center_x = 0.0

live_view_center_y = 0.0

map_min_x = -MAP_HALF_SIZE_M

map_max_x = MAP_HALF_SIZE_M

map_min_y = -MAP_HALF_SIZE_M

map_max_y = MAP_HALF_SIZE_M


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


def heading_difference(
        first,
        second):

    return normalize_heading(
        first
        -
        second
    )


# ============================================================
# ROTATION MATRIX
#
# +X = right
# +Y = forward
#
# 0° = forward
# ============================================================

def heading_rotation_matrix(
        heading_deg):

    heading_rad = math.radians(
        heading_deg
    )

    cos_h = math.cos(
        heading_rad
    )

    sin_h = math.sin(
        heading_rad
    )

    return np.array(
        [
            [
                cos_h,
                sin_h
            ],
            [
                -sin_h,
                cos_h
            ]
        ],
        dtype=float
    )


# ============================================================
# READ NAVX
#
# Expected Java NetworkTables:
#
# table:
# navX RobotPose table
#
# key:
# Heading
# ============================================================

def read_navx_heading():

    global raw_navx_heading
    global navx_available

    value = robot_pose_table.getNumber(
        "Heading",
        9999.0
    )

    if value == 9999.0:

        navx_available = False

        return None

    raw_navx_heading = float(
        value
    )

    navx_available = True

    return raw_navx_heading


# ============================================================
# GET PYTHON-RELATIVE NAVX HEADING
# ============================================================

def get_relative_navx_heading():

    global navx_zero_offset
    global navx_zero_initialized

    current_raw = read_navx_heading()

    if current_raw is None:

        return None

    if not navx_zero_initialized:

        navx_zero_offset = (
            current_raw
        )

        navx_zero_initialized = True

    relative = (
        current_raw
        -
        navx_zero_offset
    )

    relative *= (
        NAVX_HEADING_SIGN
    )

    return normalize_heading(
        relative
    )


# ============================================================
# RESET PYTHON NAVX ZERO
# ============================================================

def reset_python_navx_zero():

    global navx_zero_offset
    global navx_zero_initialized

    global robot_heading
    global previous_navx_heading
    global keyframe_navx_heading

    current_raw = read_navx_heading()

    if current_raw is None:

        navx_zero_initialized = False

        robot_heading = 0.0

        previous_navx_heading = 0.0

        keyframe_navx_heading = 0.0

        return

    navx_zero_offset = (
        current_raw
    )

    navx_zero_initialized = True

    robot_heading = 0.0

    previous_navx_heading = 0.0

    keyframe_navx_heading = 0.0


# ============================================================
# KEYBOARD MOTION STATE
#
# These helpers are used ONLY to decide whether LiDAR X/Y
# translation should be estimated.
#
# Pure Q/E:
#   X/Y HOLD
#   navX heading changes
#
# W/S/A/D:
#   X/Y tracked by LiDAR ICP
#
# W+Q / W+E / S+Q / S+E / A/D+Q/E:
#   X/Y tracked by LiDAR ICP
#   navX heading changes at the same time
#   -> curved route
# ============================================================

def get_keyboard_command_state():

    enabled = keyboard_table.getBoolean(
        "Enabled",
        False
    )

    command_x = keyboard_table.getNumber(
        "X",
        0.0
    )

    command_y = keyboard_table.getNumber(
        "Y",
        0.0
    )

    command_z = keyboard_table.getNumber(
        "Z",
        0.0
    )

    translation = (
        enabled
        and
        (
            abs(command_x)
            >=
            KEYBOARD_MOVE_THRESHOLD
            or
            abs(command_y)
            >=
            KEYBOARD_MOVE_THRESHOLD
        )
    )

    rotation = (
        enabled
        and
        abs(command_z)
        >=
        KEYBOARD_TURN_THRESHOLD
    )

    return (
        enabled,
        float(command_x),
        float(command_y),
        float(command_z),
        translation,
        rotation
    )


def translation_commanded():

    (
        _,
        _,
        _,
        _,
        translation,
        _
    ) = get_keyboard_command_state()

    return translation


def rotation_commanded():

    (
        _,
        _,
        _,
        _,
        _,
        rotation
    ) = get_keyboard_command_state()

    return rotation


def pure_rotation_commanded():

    (
        enabled,
        _,
        _,
        _,
        translation,
        rotation
    ) = get_keyboard_command_state()

    return (
        enabled
        and
        rotation
        and
        not translation
    )


def stationary_commanded():

    (
        enabled,
        _,
        _,
        _,
        translation,
        rotation
    ) = get_keyboard_command_state()

    return (
        enabled
        and
        not translation
        and
        not rotation
    )


# ============================================================
# MAPPING POSE FROM LIDAR LOCALIZATION + NAVX
#
# IMPORTANT:
#
# X/Y come from this file's LiDAR ICP localization:
#     robot_x
#     robot_y
#
# Heading comes from navX:
#     robot_heading
#
# No encoder X/Y is used here.
#
# We only subtract the mapping origin. We do NOT rotate the
# already-localized X/Y by the reset heading again.
# ============================================================

def read_mapping_pose():

    global map_pose_zero_initialized
    global map_pose_zero_x
    global map_pose_zero_y
    global map_pose_zero_heading

    global mapping_robot_x
    global mapping_robot_y
    global mapping_robot_heading
    global mapping_pose_available

    # Mapping is trusted only while the LiDAR translation and navX
    # heading are both valid.
    if (
        not localization_valid
        or
        not navx_available
    ):

        mapping_pose_available = False

        return False

    if not map_pose_zero_initialized:

        map_pose_zero_x = float(
            robot_x
        )

        map_pose_zero_y = float(
            robot_y
        )

        map_pose_zero_heading = float(
            robot_heading
        )

        map_pose_zero_initialized = True

    # LiDAR localization is already in a world frame.
    # Only remove the map origin.
    delta_world_x = (
        robot_x
        -
        map_pose_zero_x
    )

    delta_world_y = (
        robot_y
        -
        map_pose_zero_y
    )

    # Fixed display/map axis conversion:
    # localization +Y forward -> map +X/right
    # localization +X right   -> map -Y/down
    mapping_robot_x = float(
        delta_world_y
    )

    mapping_robot_y = float(
        -delta_world_x
    )

    mapping_robot_heading = normalize_heading(
        robot_heading
        -
        map_pose_zero_heading
    )

    mapping_pose_available = True

    return True


def reset_mapping_pose_zero():

    global map_pose_zero_initialized
    global map_pose_zero_x
    global map_pose_zero_y
    global map_pose_zero_heading

    global mapping_robot_x
    global mapping_robot_y
    global mapping_robot_heading
    global mapping_pose_available

    # reset_localization() has already reset the LiDAR translation
    # and the Python navX reference. Use those exact values as the
    # new fixed-map origin.
    map_pose_zero_x = float(
        robot_x
    )

    map_pose_zero_y = float(
        robot_y
    )

    map_pose_zero_heading = float(
        robot_heading
    )

    map_pose_zero_initialized = True

    mapping_robot_x = 0.0

    mapping_robot_y = 0.0

    mapping_robot_heading = 0.0

    mapping_pose_available = (
        navx_available
    )


# ============================================================
# LOCAL LIDAR POINT -> FIXED MAP COORDINATE
# ============================================================

def local_points_to_mapping_world(
        local_points):

    if len(local_points) == 0:

        return np.empty(
            (0, 2)
        )

    heading_rad = math.radians(
        mapping_robot_heading
    )

    cos_h = math.cos(
        heading_rad
    )

    sin_h = math.sin(
        heading_rad
    )

    local_x = local_points[:, 0]

    local_y = local_points[:, 1]

    # Display/map convention:
    # 0° = +X/right
    # positive navX = clockwise
    map_delta_x = (
        -local_x
        *
        sin_h
        +
        local_y
        *
        cos_h
    )

    map_delta_y = (
        -local_x
        *
        cos_h
        -
        local_y
        *
        sin_h
    )

    map_x = (
        mapping_robot_x
        +
        map_delta_x
    )

    map_y = (
        mapping_robot_y
        +
        map_delta_y
    )

    return np.column_stack(
        (
            map_x,
            map_y
        )
    )


# ============================================================
# LOCALIZATION WORLD -> DISPLAY COORDINATES
#
# LiDAR localization internal frame:
# +X = right at heading 0
# +Y = forward at heading 0
#
# Display frame:
# +X = forward at heading 0
# +Y = left at heading 0
#
# This is a FIXED axis conversion only.
# ============================================================

def localization_world_points_to_display(
        world_points):

    if len(world_points) == 0:

        return np.empty(
            (0, 2)
        )

    display_x = world_points[:, 1]

    display_y = -world_points[:, 0]

    return np.column_stack(
        (
            display_x,
            display_y
        )
    )


def localization_world_pose_to_display(
        world_x,
        world_y):

    return (
        float(world_y),
        float(-world_x)
    )


# ============================================================
# PARSE SCAN
# ============================================================

def parse_scan(
        scan_array):

    angles = []

    distances = []

    if scan_array is None:

        return (
            angles,
            distances
        )

    usable_length = (
        len(scan_array)
        -
        len(scan_array) % 2
    )

    for index in range(
        0,
        usable_length,
        2
    ):

        raw_angle = float(
            scan_array[index]
        )

        raw_distance_mm = float(
            scan_array[index + 1]
        )

        if (
            raw_distance_mm
            >=
            MIN_DISTANCE_MM
            and
            raw_distance_mm
            <=
            MAX_DISTANCE_MM
        ):

            corrected_angle = normalize_angle(
                raw_angle
                +
                LIDAR_MOUNT_OFFSET_DEG
            )

            corrected_distance = (
                raw_distance_mm
                *
                DISTANCE_SCALE_FACTOR
            )

            angles.append(
                corrected_angle
            )

            distances.append(
                corrected_distance
            )

    return (
        angles,
        distances
    )


# ============================================================
# GET FULL SCAN
# ============================================================

def get_full_scan():

    scan_a = lidar_table.getNumberArray(
        "ScanA",
        []
    )

    scan_b = lidar_table.getNumberArray(
        "ScanB",
        []
    )

    angles_a, distances_a = parse_scan(
        scan_a
    )

    angles_b, distances_b = parse_scan(
        scan_b
    )

    angles = (
        angles_a
        +
        angles_b
    )

    distances = (
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

    distances = np.array(
        distances,
        dtype=float
    )

    order = np.argsort(
        angles
    )

    return (
        angles[order],
        distances[order]
    )


# ============================================================
# POLAR -> LOCAL XY
# ============================================================

def scan_to_xy(
        angles,
        distances):

    if len(angles) == 0:

        return np.empty(
            (0, 2)
        )

    angle_rad = np.radians(
        angles
    )

    distance_m = (
        distances
        /
        1000.0
    )

    local_x = (
        distance_m
        *
        np.sin(
            angle_rad
        )
    )

    local_y = (
        distance_m
        *
        np.cos(
            angle_rad
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
        maximum):

    if len(points) <= maximum:

        return points.copy()

    indices = np.linspace(
        0,
        len(points) - 1,
        maximum,
        dtype=int
    )

    return points[
        indices
    ]


# ============================================================
# TRANSLATION-ONLY ICP
#
# navX supplies the rotation.
# ICP estimates translation only.
# ============================================================

def translation_icp(
        current_points,
        reference_points,
        known_heading_delta_deg,
        max_points,
        iterations,
        correspondence_distance,
        minimum_matches):

    current = downsample_points(
        current_points,
        max_points
    )

    reference = downsample_points(
        reference_points,
        max_points
    )

    if (
        len(current)
        <
        minimum_matches
        or
        len(reference)
        <
        minimum_matches
    ):

        return (
            np.zeros(2),
            False,
            999.0,
            0.0
        )

    # ========================================================
    # ALIGN CURRENT SCAN USING NAVX ROTATION
    # ========================================================

    known_rotation = heading_rotation_matrix(
        known_heading_delta_deg
    )

    transformed = (
        current
        @
        known_rotation.T
    )

    total_translation = np.zeros(
        2
    )

    tree = cKDTree(
        reference
    )

    previous_error = None

    final_ratio = 0.0

    for _ in range(
        iterations
    ):

        distances, indices = tree.query(
            transformed,
            k=1
        )

        valid = (
            distances
            <
            correspondence_distance
        )

        valid_count = int(
            np.count_nonzero(
                valid
            )
        )

        if (
            valid_count
            <
            minimum_matches
        ):

            return (
                np.zeros(2),
                False,
                999.0,
                0.0
            )

        final_ratio = (
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

        translation_step = np.mean(
            target_match
            -
            source_match,
            axis=0
        )

        transformed = (
            transformed
            +
            translation_step
        )

        total_translation = (
            total_translation
            +
            translation_step
        )

        new_distances, _ = tree.query(
            transformed,
            k=1
        )

        new_valid = (
            new_distances
            <
            correspondence_distance
        )

        if (
            np.count_nonzero(
                new_valid
            )
            <
            minimum_matches
        ):

            return (
                np.zeros(2),
                False,
                999.0,
                0.0
            )

        error = float(
            np.mean(
                new_distances[
                    new_valid
                ]
            )
        )

        if previous_error is not None:

            improvement = abs(
                previous_error
                -
                error
            )

            if improvement < 0.0001:

                previous_error = (
                    error
                )

                break

        previous_error = (
            error
        )

    if previous_error is None:

        return (
            np.zeros(2),
            False,
            999.0,
            0.0
        )

    return (
        total_translation,
        True,
        previous_error,
        final_ratio
    )


# ============================================================
# NORMAL TRANSLATION MATCH
# ============================================================

def normal_translation_match(
        current,
        reference,
        heading_delta):

    return translation_icp(
        current,
        reference,
        heading_delta,

        NORMAL_ICP_POINTS,
        NORMAL_ICP_ITERATIONS,
        NORMAL_CORRESPONDENCE_M,
        NORMAL_MIN_MATCHES
    )


# ============================================================
# RECOVERY TRANSLATION MATCH
# ============================================================

def recovery_translation_match(
        current,
        reference,
        heading_delta):

    return translation_icp(
        current,
        reference,
        heading_delta,

        RECOVERY_ICP_POINTS,
        RECOVERY_ICP_ITERATIONS,
        RECOVERY_CORRESPONDENCE_M,
        RECOVERY_MIN_MATCHES
    )


# ============================================================
# TRANSLATION QUALITY
# ============================================================

def translation_quality_good(
        translation,
        error,
        inlier_ratio,
        recovery=False):

    movement = float(
        np.linalg.norm(
            translation
        )
    )

    if recovery:

        return (
            error
            <=
            RECOVERY_MAX_ERROR_M
            and
            inlier_ratio
            >=
            RECOVERY_MIN_INLIER_RATIO
            and
            movement
            <=
            RECOVERY_MAX_TRANSLATION_M
        )

    return (
        error
        <=
        NORMAL_MAX_ERROR_M
        and
        inlier_ratio
        >=
        NORMAL_MIN_INLIER_RATIO
        and
        movement
        <=
        NORMAL_MAX_TRANSLATION_M
    )


# ============================================================
# LOCAL TRANSLATION -> WORLD
# ============================================================

def local_translation_to_world(
        local_translation,
        reference_heading):

    heading_rad = math.radians(
        reference_heading
    )

    cos_h = math.cos(
        heading_rad
    )

    sin_h = math.sin(
        heading_rad
    )

    local_x = float(
        local_translation[0]
    )

    local_y = float(
        local_translation[1]
    )

    world_x = (
        local_x
        *
        cos_h
        +
        local_y
        *
        sin_h
    )

    world_y = (
        -local_x
        *
        sin_h
        +
        local_y
        *
        cos_h
    )

    return np.array(
        [
            world_x,
            world_y
        ],
        dtype=float
    )


# ============================================================
# STORE PREVIOUS GOOD SCAN
# ============================================================

def store_previous_good_scan(
        scan):

    global previous_good_scan
    global previous_good_translation
    global previous_navx_heading

    previous_good_scan = (
        scan.copy()
    )

    previous_good_translation = (
        robot_translation.copy()
    )

    previous_navx_heading = (
        robot_heading
    )


# ============================================================
# CREATE KEYFRAME
# ============================================================

def create_keyframe(
        scan):

    global keyframe_scan
    global keyframe_translation
    global keyframe_navx_heading
    global keyframe_number

    keyframe_scan = (
        scan.copy()
    )

    keyframe_translation = (
        robot_translation.copy()
    )

    keyframe_navx_heading = (
        robot_heading
    )

    keyframe_number += 1


# ============================================================
# KEYFRAME NEEDED?
# ============================================================

def keyframe_needed():

    movement = float(
        np.linalg.norm(
            robot_translation
            -
            keyframe_translation
        )
    )

    heading_change = abs(
        heading_difference(
            robot_heading,
            keyframe_navx_heading
        )
    )

    return (
        movement
        >=
        KEYFRAME_TRANSLATION_M
        or
        heading_change
        >=
        KEYFRAME_ROTATION_DEG
    )


# ============================================================
# LOCALIZATION
#
# navX = heading
# LiDAR = translation
# ============================================================

def track_localization(
        current_scan):

    global robot_x
    global robot_y
    global robot_heading
    global robot_translation

    global previous_good_scan
    global previous_good_translation
    global previous_navx_heading

    global localization_valid
    global localization_state
    global pose_locked

    global recovery_count
    global rotation_hold_count
    global consecutive_hold_count

    global last_icp_error
    global last_inlier_ratio
    global last_relative_translation
    global last_navx_delta


    # ========================================================
    # READ NAVX
    # ========================================================

    current_heading = (
        get_relative_navx_heading()
    )

    if current_heading is None:

        localization_valid = False

        localization_state = (
            "NO_NAVX"
        )

        consecutive_hold_count += 1

        return


    # ========================================================
    # NAVX IS THE HEADING AUTHORITY
    # ========================================================

    robot_heading = (
        current_heading
    )


    # ========================================================
    # FIRST VALID SCAN
    # ========================================================

    if previous_good_scan is None:

        robot_translation = np.array(
            [
                0.0,
                0.0
            ],
            dtype=float
        )

        robot_x = 0.0

        robot_y = 0.0

        localization_valid = True

        localization_state = (
            "INITIALIZED"
        )

        pose_locked = True

        consecutive_hold_count = 0

        last_relative_translation = 0.0

        store_previous_good_scan(
            current_scan
        )

        create_keyframe(
            current_scan
        )

        return


    # ========================================================
    # HEADING CHANGE SINCE PREVIOUS REFERENCE SCAN
    # ========================================================

    navx_delta = heading_difference(
        robot_heading,
        previous_navx_heading
    )

    last_navx_delta = (
        navx_delta
    )


    # ========================================================
    # READ CURRENT KEYBOARD MOTION STATE
    # ========================================================

    (
        keyboard_enabled,
        command_x,
        command_y,
        command_z,
        translation_is_commanded,
        rotation_is_commanded
    ) = get_keyboard_command_state()


    # ========================================================
    # PURE ROTATION / IDLE X-Y HOLD
    #
    # THIS IS THE IMPORTANT FIX.
    #
    # Q / E only:
    #   - DO NOT run translation ICP
    #   - DO NOT change robot_x / robot_y
    #   - heading still follows navX
    #   - refresh reference scan every frame
    #
    # Idle:
    #   - same X/Y hold
    #   - refresh reference scan every frame
    #
    # Because the latest scan becomes the new reference while
    # rotating, when W/A/S/D is pressed after the turn there is
    # no old pre-turn scan waiting to create a position jump.
    # ========================================================

    if (
        keyboard_enabled
        and
        not translation_is_commanded
    ):

        localization_valid = True

        pose_locked = True

        last_relative_translation = 0.0

        last_icp_error = 0.0

        last_inlier_ratio = 1.0

        consecutive_hold_count = 0

        if rotation_is_commanded:

            localization_state = (
                "ROTATION_XY_HOLD"
            )

            rotation_hold_count += 1

        else:

            localization_state = (
                "IDLE_XY_HOLD"
            )


        # ----------------------------------------------------
        # IMPORTANT:
        #
        # Keep the same X/Y translation but refresh the scan
        # and heading reference.
        # ----------------------------------------------------

        previous_good_scan = (
            current_scan.copy()
        )

        previous_good_translation = (
            robot_translation.copy()
        )

        previous_navx_heading = (
            robot_heading
        )


        # ----------------------------------------------------
        # Refresh keyframe when heading moved enough.
        #
        # Translation stays unchanged.
        # ----------------------------------------------------

        if (
            abs(
                heading_difference(
                    robot_heading,
                    keyframe_navx_heading
                )
            )
            >=
            KEYFRAME_ROTATION_DEG
        ):

            create_keyframe(
                current_scan
            )

        return


    # ========================================================
    # IF KEYBOARD IS DISABLED
    #
    # We do not assume the robot is stationary because it could
    # be pushed manually. LiDAR is allowed to estimate motion.
    # ========================================================


    # ========================================================
    # NORMAL / COMBINED MOTION
    #
    # Translation commands include:
    #
    # W, S, A, D
    # W+Q, W+E
    # S+Q, S+E
    # A/D + Q/E
    #
    # navX supplies the rotation.
    # LiDAR ICP supplies translation.
    # ========================================================

    (
        relative_translation,
        success,
        error,
        inlier_ratio
    ) = normal_translation_match(
        current_scan,
        previous_good_scan,
        navx_delta
    )

    last_icp_error = (
        error
    )

    last_inlier_ratio = (
        inlier_ratio
    )

    normal_good = (
        success
        and
        translation_quality_good(
            relative_translation,
            error,
            inlier_ratio,
            False
        )
    )


    # ========================================================
    # NORMAL ICP SUCCESS
    # ========================================================

    if normal_good:

        # ----------------------------------------------------
        # TRANSLATION DIRECTION
        #
        # For THIS LiDAR / coordinate convention, the ICP
        # translation already matches the robot-motion direction
        # used by the existing localization/display frame.
        #
        # Using the negative value makes:
        #     W -> appear backward
        #     S -> appear forward
        #
        # Therefore keep the translation direction directly.
        # ----------------------------------------------------

        local_robot_movement = (
            relative_translation
        )

        relative_distance = float(
            np.linalg.norm(
                local_robot_movement
            )
        )

        last_relative_translation = (
            relative_distance
        )


        # ----------------------------------------------------
        # MIDPOINT HEADING
        #
        # During W+Q / W+E the robot translates while turning.
        # Using the heading halfway through the scan interval
        # gives a smoother curved trajectory.
        # ----------------------------------------------------

        midpoint_heading = normalize_heading(
            previous_navx_heading
            +
            (
                navx_delta
                *
                0.5
            )
        )


        world_delta = local_translation_to_world(
            local_robot_movement,
            midpoint_heading
        )


        # ----------------------------------------------------
        # MICRO-MOVEMENT FILTER
        # ----------------------------------------------------

        if (
            np.linalg.norm(
                world_delta
            )
            <
            STATIONARY_TRANSLATION_M
        ):

            world_delta = np.array(
                [
                    0.0,
                    0.0
                ],
                dtype=float
            )

            pose_locked = True

        else:

            pose_locked = False


        robot_translation = (
            previous_good_translation
            +
            world_delta
        )

        robot_x = float(
            robot_translation[0]
        )

        robot_y = float(
            robot_translation[1]
        )

        localization_valid = True

        localization_state = (
            "TRACKING_TURN"
            if rotation_is_commanded
            else
            "TRACKING"
        )

        consecutive_hold_count = 0


        # ----------------------------------------------------
        # STORE CURRENT SCAN AS NEXT REFERENCE
        # ----------------------------------------------------

        store_previous_good_scan(
            current_scan
        )


        if keyframe_needed():

            create_keyframe(
                current_scan
            )

        return


    # ========================================================
    # NORMAL FAILED -> KEYFRAME RECOVERY
    # ========================================================

    if keyframe_scan is not None:

        keyframe_heading_delta = (
            heading_difference(
                robot_heading,
                keyframe_navx_heading
            )
        )

        (
            recovery_translation,
            recovery_success,
            recovery_error,
            recovery_ratio
        ) = recovery_translation_match(
            current_scan,
            keyframe_scan,
            keyframe_heading_delta
        )

        recovery_good = (
            recovery_success
            and
            translation_quality_good(
                recovery_translation,
                recovery_error,
                recovery_ratio,
                True
            )
        )

        if recovery_good:

            # Keep the SAME direction convention as normal ICP.
            #
            # This is important so a recovery frame cannot
            # suddenly reverse the route after normal tracking.
            local_robot_movement = (
                recovery_translation
            )

            recovery_distance = float(
                np.linalg.norm(
                    local_robot_movement
                )
            )

            last_relative_translation = (
                recovery_distance
            )


            # Keyframe pose is the world reference for this
            # recovery translation.
            world_delta = local_translation_to_world(
                local_robot_movement,
                keyframe_navx_heading
            )


            robot_translation = (
                keyframe_translation
                +
                world_delta
            )

            robot_x = float(
                robot_translation[0]
            )

            robot_y = float(
                robot_translation[1]
            )

            localization_valid = True

            localization_state = (
                "RECOVERED_TURN"
                if rotation_is_commanded
                else
                "RECOVERED"
            )

            pose_locked = False

            recovery_count += 1

            consecutive_hold_count = 0

            last_icp_error = (
                recovery_error
            )

            last_inlier_ratio = (
                recovery_ratio
            )

            store_previous_good_scan(
                current_scan
            )

            create_keyframe(
                current_scan
            )

            return


    # ========================================================
    # ICP FAILED
    #
    # Keep X/Y exactly where it was.
    #
    # We deliberately do NOT invent movement from a failed
    # match.
    # ========================================================

    localization_valid = False

    localization_state = (
        "XY_HOLD_BAD_ICP"
    )

    pose_locked = True

    last_relative_translation = 0.0

    consecutive_hold_count += 1


    # --------------------------------------------------------
    # After several failed frames, refresh the previous scan.
    #
    # This prevents one bad reference from trapping the system
    # forever, but X/Y remains unchanged during the refresh.
    # --------------------------------------------------------

    if consecutive_hold_count >= 3:

        previous_good_scan = (
            current_scan.copy()
        )

        previous_good_translation = (
            robot_translation.copy()
        )

        previous_navx_heading = (
            robot_heading
        )

        consecutive_hold_count = 0


# ============================================================
# LOCAL POINTS -> WORLD
#
# Uses navX heading.
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
# LIVE CLUSTER FILTER
# ============================================================

def find_live_clusters(
        local_points):

    clusters = []

    if len(local_points) == 0:

        return clusters

    current_cluster = [
        local_points[0]
    ]

    for index in range(
        1,
        len(local_points)
    ):

        gap = np.linalg.norm(
            local_points[index]
            -
            local_points[index - 1]
        )

        if (
            gap
            <=
            LIVE_CLUSTER_GAP_M
        ):

            current_cluster.append(
                local_points[index]
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
                local_points[index]
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


def clusters_to_points(
        clusters):

    if len(clusters) == 0:

        return np.empty(
            (0, 2)
        )

    return np.vstack(
        clusters
    )


# ============================================================
# OCCUPANCY GRID HELPERS
# ============================================================

def map_xy_to_grid(
        x,
        y):

    column = int(
        math.floor(
            (
                x
                +
                MAP_HALF_SIZE_M
            )
            /
            GRID_SIZE_M
        )
    )

    row = int(
        math.floor(
            (
                y
                +
                MAP_HALF_SIZE_M
            )
            /
            GRID_SIZE_M
        )
    )

    return (
        row,
        column
    )


def grid_cell_inside(
        row,
        column):

    return (
        row >= 0
        and
        row < MAP_CELL_COUNT
        and
        column >= 0
        and
        column < MAP_CELL_COUNT
    )


def add_free_evidence(
        row,
        column):

    if not grid_cell_inside(
        row,
        column
    ):

        return

    occupancy_grid[
        row,
        column
    ] = max(
        MIN_CELL_CONFIDENCE,
        int(
            occupancy_grid[
                row,
                column
            ]
        )
        +
        FREE_CELL_UPDATE
    )


def add_occupied_evidence(
        row,
        column):

    if not grid_cell_inside(
        row,
        column
    ):

        return

    occupancy_grid[
        row,
        column
    ] = min(
        MAX_CELL_CONFIDENCE,
        int(
            occupancy_grid[
                row,
                column
            ]
        )
        +
        OCCUPIED_CELL_UPDATE
    )


# ============================================================
# UPDATE ONE LIDAR RAY
#
# Robot -> endpoint:
#
# cells before endpoint = FREE
# endpoint = OCCUPIED
# ============================================================

def update_one_occupancy_ray(
        robot_map_x,
        robot_map_y,
        endpoint_x,
        endpoint_y):

    start_row, start_column = map_xy_to_grid(
        robot_map_x,
        robot_map_y
    )

    end_row, end_column = map_xy_to_grid(
        endpoint_x,
        endpoint_y
    )

    delta_column = (
        end_column
        -
        start_column
    )

    delta_row = (
        end_row
        -
        start_row
    )

    steps = int(
        max(
            abs(
                delta_column
            ),
            abs(
                delta_row
            )
        )
    )

    if steps <= 0:

        if grid_cell_inside(
            end_row,
            end_column
        ):

            add_occupied_evidence(
                end_row,
                end_column
            )

        return

    # Free-space ray. Do not include the final endpoint cell.
    for step in range(
        0,
        steps
    ):

        fraction = (
            float(step)
            /
            float(steps)
        )

        column = int(
            round(
                start_column
                +
                delta_column
                *
                fraction
            )
        )

        row = int(
            round(
                start_row
                +
                delta_row
                *
                fraction
            )
        )

        add_free_evidence(
            row,
            column
        )

    # Only mark an obstacle if its measured endpoint is actually
    # inside the fixed map. A ray whose endpoint is outside the
    # 7 m map still clears free cells that pass through the map.
    if grid_cell_inside(
        end_row,
        end_column
    ):

        add_occupied_evidence(
            end_row,
            end_column
        )


# ============================================================
# OCCUPANCY GRID UPDATE
# ============================================================

def update_occupancy_grid(
        local_points):

    if (
        not mapping_pose_available
        or
        len(local_points) == 0
    ):

        return

    map_points = local_points_to_mapping_world(
        local_points
    )

    for index in range(
        0,
        len(map_points),
        MAP_RAY_STRIDE
    ):

        endpoint = map_points[
            index
        ]

        update_one_occupancy_ray(
            mapping_robot_x,
            mapping_robot_y,
            float(
                endpoint[0]
            ),
            float(
                endpoint[1]
            )
        )


def get_confirmed_map_points():

    rows, columns = np.where(
        occupancy_grid
        >=
        OCCUPIED_CONFIRM_THRESHOLD
    )

    if len(rows) == 0:

        return np.empty(
            (0, 2)
        )

    x_values = (
        (
            columns.astype(float)
            +
            0.5
        )
        *
        GRID_SIZE_M
        -
        MAP_HALF_SIZE_M
    )

    y_values = (
        (
            rows.astype(float)
            +
            0.5
        )
        *
        GRID_SIZE_M
        -
        MAP_HALF_SIZE_M
    )

    return np.column_stack(
        (
            x_values,
            y_values
        )
    )


def get_occupancy_display_image():

    image = np.full(
        occupancy_grid.shape,
        0.55,
        dtype=float
    )

    free_mask = (
        occupancy_grid
        <=
        FREE_CELL_THRESHOLD
    )

    occupied_mask = (
        occupancy_grid
        >=
        OCCUPIED_CONFIRM_THRESHOLD
    )

    image[
        free_mask
    ] = 1.0

    image[
        occupied_mask
    ] = 0.0

    return image


def get_map_coverage_percent():

    known_cells = int(
        np.count_nonzero(
            occupancy_grid
            !=
            0
        )
    )

    total_cells = int(
        occupancy_grid.size
    )

    if total_cells <= 0:

        return 0.0

    return (
        100.0
        *
        known_cells
        /
        total_cells
    )


# ============================================================
# NEAREST OBJECT
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

    angle = math.degrees(
        math.atan2(
            local_point[0],
            local_point[1]
        )
    )

    return (
        float(
            world_point[0]
        ),

        float(
            world_point[1]
        ),

        float(
            distances[index]
        ),

        normalize_angle(
            angle
        )
    )


# ============================================================
# LOG POSITION
# ============================================================

def log_current_position():

    global position_counter
    global last_log_message

    if not navx_available:

        last_log_message = (
            "LOG FAILED - NAVX NOT AVAILABLE"
        )

        print(
            last_log_message
        )

        return

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

    position = {
        "name": position_name,
        "x": float(robot_x),
        "y": float(robot_y),
        "heading": float(robot_heading),
        "timestamp": timestamp
    }

    logged_positions.append(
        position
    )

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
        f"{position_name} "
        f"X={robot_x:.2f} "
        f"Y={robot_y:.2f} "
        f"H={robot_heading:.1f}°"
    )

    print(
        last_log_message
    )


# ============================================================
# NETWORKTABLE OUTPUT
# ============================================================

def publish_localization(
        obstacle_x,
        obstacle_y,
        obstacle_distance,
        obstacle_angle):

    global localization_sample_id

    localization_sample_id += 1

    localization_table.putNumber(
        "SampleId",
        localization_sample_id
    )

    localization_table.putNumber(
        "ResetId",
        localization_reset_id
    )

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
        "NavXRawHeading",
        raw_navx_heading
    )

    localization_table.putNumber(
        "NavXZeroOffset",
        navx_zero_offset
    )

    localization_table.putNumber(
        "NavXDelta",
        last_navx_delta
    )

    localization_table.putBoolean(
        "NavXAvailable",
        navx_available
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

    localization_table.putString(
        "LocalizationState",
        localization_state
    )

    localization_table.putBoolean(
        "LocalizationValid",
        localization_valid
    )

    localization_table.putNumber(
        "RecoveryCount",
        recovery_count
    )

    localization_table.putNumber(
        "RotationHoldCount",
        rotation_hold_count
    )

    localization_table.putNumber(
        "HoldCount",
        consecutive_hold_count
    )

    localization_table.putNumber(
        "ICPError",
        last_icp_error
    )

    localization_table.putNumber(
        "ICPInlierRatio",
        last_inlier_ratio
    )


# ============================================================
# SAVE MAP
# ============================================================

def save_map():

    points = get_confirmed_map_points()

    timestamp = time.strftime(
        "%Y%m%d_%H%M%S"
    )

    csv_path = os.path.join(
        MAP_SAVE_FOLDER,
        "lidar_occupancy_map_"
        +
        timestamp
        +
        ".csv"
    )

    png_path = os.path.join(
        MAP_SAVE_FOLDER,
        "lidar_occupancy_map_"
        +
        timestamp
        +
        ".png"
    )

    npy_path = os.path.join(
        MAP_SAVE_FOLDER,
        "lidar_occupancy_grid_"
        +
        timestamp
        +
        ".npy"
    )

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

        for point in points:

            writer.writerow(
                [
                    point[0],
                    point[1]
                ]
            )

    np.save(
        npy_path,
        occupancy_grid
    )

    save_fig, save_axis = plt.subplots(
        figsize=(9, 9)
    )

    save_axis.imshow(
        get_occupancy_display_image(),
        origin="lower",
        extent=[
            -MAP_HALF_SIZE_M,
            MAP_HALF_SIZE_M,
            -MAP_HALF_SIZE_M,
            MAP_HALF_SIZE_M
        ],
        cmap="gray",
        vmin=0.0,
        vmax=1.0,
        interpolation="nearest"
    )

    # Logged positions are stored in the LiDAR-localization
    # coordinate convention (+Y forward). Convert them to the
    # same display convention used by the occupancy map.
    for position in logged_positions:

        display_x = position["y"]

        display_y = -position["x"]

        save_axis.scatter(
            display_x,
            display_y,
            s=60,
            marker="o"
        )

        save_axis.text(
            display_x,
            display_y + 0.12,
            (
                f'{position["name"]}\n'
                f'{display_x:.2f}, '
                f'{display_y:.2f}\n'
                f'{position["heading"]:.1f}°'
            ),
            ha="center"
        )

    save_axis.scatter(
        mapping_robot_x,
        mapping_robot_y,
        s=100,
        marker="o"
    )

    heading_rad = math.radians(
        mapping_robot_heading
    )

    save_axis.arrow(
        mapping_robot_x,
        mapping_robot_y,
        0.35
        *
        math.cos(
            heading_rad
        ),
        -0.35
        *
        math.sin(
            heading_rad
        ),
        width=0.01,
        head_width=0.10,
        length_includes_head=True
    )

    save_axis.set_xlim(
        -MAP_HALF_SIZE_M,
        MAP_HALF_SIZE_M
    )

    save_axis.set_ylim(
        -MAP_HALF_SIZE_M,
        MAP_HALF_SIZE_M
    )

    save_axis.set_aspect(
        "equal",
        adjustable="box"
    )

    save_axis.set_xlabel(
        "Map X (m)"
    )

    save_axis.set_ylabel(
        "Map Y (m)"
    )

    save_axis.set_title(
        f"LiDAR Occupancy Map - {MAP_SIZE_M:.1f} m x {MAP_SIZE_M:.1f} m"
    )

    save_fig.savefig(
        png_path,
        dpi=200,
        bbox_inches="tight"
    )

    plt.close(
        save_fig
    )

    print(
        "OCCUPANCY MAP SAVED"
    )

    print(
        csv_path
    )

    print(
        png_path
    )

    print(
        npy_path
    )


# ============================================================
# RESET
#
# R:
#
# X = 0
# Y = 0
# current heading = 0°
# clear map
# clear scan references
# ============================================================

def reset_localization():

    global localization_reset_id

    global robot_x
    global robot_y
    global robot_heading
    global robot_translation

    global previous_good_scan
    global previous_good_translation
    global previous_navx_heading

    global keyframe_scan
    global keyframe_translation
    global keyframe_navx_heading
    global keyframe_number

    global localization_valid
    global localization_state
    global pose_locked

    global recovery_count
    global rotation_hold_count
    global consecutive_hold_count

    global frame_counter

    global last_icp_error
    global last_inlier_ratio
    global last_relative_translation
    global last_navx_delta

    global occupancy_grid

    global map_pose_zero_initialized
    global map_pose_zero_x
    global map_pose_zero_y
    global map_pose_zero_heading

    global mapping_robot_x
    global mapping_robot_y
    global mapping_robot_heading
    global mapping_pose_available


    # ========================================================
    # RESET ROBOT POSITION
    # ========================================================

    robot_x = 0.0

    robot_y = 0.0

    robot_heading = 0.0

    robot_translation = np.array(
        [
            0.0,
            0.0
        ],
        dtype=float
    )


    # ========================================================
    # RESET NAVX ZERO
    # ========================================================

    reset_python_navx_zero()


    # ========================================================
    # RESET SCAN REFERENCES
    # ========================================================

    previous_good_scan = None

    previous_good_translation = np.array(
        [
            0.0,
            0.0
        ],
        dtype=float
    )

    previous_navx_heading = 0.0


    # ========================================================
    # RESET KEYFRAME
    # ========================================================

    keyframe_scan = None

    keyframe_translation = np.array(
        [
            0.0,
            0.0
        ],
        dtype=float
    )

    keyframe_navx_heading = 0.0

    keyframe_number = 0


    # ========================================================
    # RESET STATUS
    # ========================================================

    localization_valid = False

    localization_state = (
        "RESETTING"
    )

    pose_locked = True

    recovery_count = 0

    rotation_hold_count = 0

    consecutive_hold_count = 0

    frame_counter = 0

    last_icp_error = 999.0

    last_inlier_ratio = 0.0

    last_relative_translation = 0.0

    last_navx_delta = 0.0

    occupancy_grid.fill(
        0
    )

    reset_mapping_pose_zero()

    localization_reset_id += 1

    localization_table.putNumber(
        "ResetId",
        localization_reset_id
    )

    localization_table.putNumber(
        "RobotX",
        0.0
    )

    localization_table.putNumber(
        "RobotY",
        0.0
    )

    localization_table.putNumber(
        "RobotHeading",
        0.0
    )

    localization_table.putString(
        "LocalizationState",
        "RESETTING"
    )


    print(
        "=========================================="
    )

    print(
        "LOCALIZATION RESET"
    )

    print(
        "X = 0"
    )

    print(
        "Y = 0"
    )

    print(
        "Current navX direction = 0 degrees"
    )

    print(
        "=========================================="
    )


# ============================================================
# MATPLOTLIB KEYS
# ============================================================

def on_key(
        event):

    if event.key == "l":

        log_current_position()

    elif event.key == "s":

        save_map()

    elif event.key == "r":

        reset_localization()


# ============================================================
# FIGURE
#
# TOP:
#
# Live | Persistent Map
#
# BOTTOM:
#
# Information
# ============================================================

plt.ion()

fig = plt.figure(
    figsize=(15, 9)
)

grid = fig.add_gridspec(
    2,
    2,
    height_ratios=[
        4.0,
        1.20
    ],
    hspace=0.24,
    wspace=0.18
)

ax_live = fig.add_subplot(
    grid[0, 0]
)

ax_map = fig.add_subplot(
    grid[0, 1]
)

ax_info = fig.add_subplot(
    grid[1, :]
)

fig.canvas.mpl_connect(
    "key_press_event",
    on_key
)

fig.suptitle(
    "LiDAR Translation + navX Heading Localization",
    fontsize=16
)

plt.show(
    block=False
)


# ============================================================
# ALLOW ALT+TAB
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
# LIVE PLOT
# ============================================================

ax_live.set_title(
    "LIVE - Current Surroundings"
)

ax_live.set_xlabel(
    "Display X (m)"
)

ax_live.set_ylabel(
    "Display Y (m)"
)

ax_live.set_aspect(
    "equal",
    adjustable="box"
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


# ============================================================
# MAP PLOT
#
# Fixed 7 m x 7 m occupancy grid:
#
# black = occupied
# white = free
# grey  = unknown
# ============================================================

ax_map.set_title(
    f"OCCUPANCY MAP - {MAP_SIZE_M:.1f} m x {MAP_SIZE_M:.1f} m"
)

ax_map.set_xlabel(
    "Map X (m)"
)

ax_map.set_ylabel(
    "Map Y (m)"
)

ax_map.set_aspect(
    "equal",
    adjustable="box"
)

ax_map.set_xlim(
    -MAP_HALF_SIZE_M,
    MAP_HALF_SIZE_M
)

ax_map.set_ylim(
    -MAP_HALF_SIZE_M,
    MAP_HALF_SIZE_M
)

map_image = ax_map.imshow(
    get_occupancy_display_image(),
    origin="lower",
    extent=[
        -MAP_HALF_SIZE_M,
        MAP_HALF_SIZE_M,
        -MAP_HALF_SIZE_M,
        MAP_HALF_SIZE_M
    ],
    cmap="gray",
    vmin=0.0,
    vmax=1.0,
    interpolation="nearest"
)

map_robot, = ax_map.plot(
    [],
    [],
    marker="o",
    linestyle="None",
    markersize=8
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
        alpha=0.90
    )
)


# ============================================================
# INFORMATION PANEL
# ============================================================

ax_info.set_axis_off()

info_text = ax_info.text(
    0.01,
    0.94,
    "",
    transform=ax_info.transAxes,
    ha="left",
    va="top",
    fontsize=9,
    family="monospace",
    bbox=dict(
        boxstyle="round,pad=0.5",
        facecolor="white",
        edgecolor="black",
        alpha=0.95
    )
)


# ============================================================
# START INFORMATION
# ============================================================

print(
    "=========================================="
)

print(
    "LIDAR + NAVX LOCALIZATION"
)

print(
    "=========================================="
)

print(
    "LiDAR = X/Y"
)

print(
    "navX = Heading"
)

print(
    "R = X0 Y0 Heading0"
)

print(
    "L = Log position"
)

print(
    "S = Save map"
)


# ============================================================
# INITIAL NAVX ZERO
# ============================================================

time.sleep(
    0.25
)

reset_python_navx_zero()

reset_mapping_pose_zero()


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while plt.fignum_exists(
        fig.number
    ):

        # ====================================================
        # READ LIDAR
        # ====================================================

        (
            angles,
            distances
        ) = get_full_scan()


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
        # LOCAL POINT CLOUD
        # ====================================================

        current_local_points = scan_to_xy(
            angles,
            distances
        )


        # ====================================================
        # LOCALIZATION
        # ====================================================

        track_localization(
            current_local_points
        )


        # ====================================================
        # FILTER ENVIRONMENT
        # ====================================================

        clusters = find_live_clusters(
            current_local_points
        )

        filtered_local_points = clusters_to_points(
            clusters
        )

        filtered_world_points = local_to_world(
            filtered_local_points
        )


        # ====================================================
        # FIXED OCCUPANCY MAP UPDATE
        #
        # Pose source:
        # LiDAR ICP = X/Y translation
        # navX      = heading
        #
        # No encoder X/Y is used for this map.
        #
        # Sensor source:
        # current filtered LiDAR points.
        # ====================================================

        read_mapping_pose()

        frame_counter += 1

        if (
            mapping_pose_available
            and
            frame_counter
            %
            MAP_UPDATE_EVERY_N_FRAMES
            ==
            0
        ):

            update_occupancy_grid(
                filtered_local_points
            )

        confirmed_points = (
            get_confirmed_map_points()
        )


        # ====================================================
        # NEAREST OBJECT
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
        # NETWORKTABLE OUTPUT
        # ====================================================

        publish_localization(
            obstacle_x,
            obstacle_y,
            obstacle_distance,
            obstacle_angle
        )


        # ====================================================
        # LIVE DISPLAY COORDINATES
        #
        # Use the exact same fixed display convention as the
        # occupancy map and keyboard route:
        #
        # 0° = +X/right
        # positive navX = clockwise
        # ====================================================

        live_display_points = (
            localization_world_points_to_display(
                filtered_world_points
            )
        )

        (
            live_display_robot_x,
            live_display_robot_y
        ) = localization_world_pose_to_display(
            robot_x,
            robot_y
        )

        heading_rad = math.radians(
            robot_heading
        )

        heading_dx = (
            0.40
            *
            math.cos(
                heading_rad
            )
        )

        heading_dy = (
            -0.40
            *
            math.sin(
                heading_rad
            )
        )


        # ====================================================
        # LIVE POINTS
        # ====================================================

        if len(
            live_display_points
        ) > 0:

            live_scatter.set_offsets(
                live_display_points
            )

        else:

            live_scatter.set_offsets(
                np.empty(
                    (0, 2)
                )
            )


        # ====================================================
        # LIVE ROBOT
        # ====================================================

        live_robot.set_data(
            [live_display_robot_x],
            [live_display_robot_y]
        )

        live_heading_line.set_data(
            [
                live_display_robot_x,
                live_display_robot_x + heading_dx
            ],
            [
                live_display_robot_y,
                live_display_robot_y + heading_dy
            ]
        )


        # ====================================================
        # LIVE RECENTER
        # ====================================================

        if (
            abs(
                live_display_robot_x
                -
                live_view_center_x
            )
            >
            LIVE_RECENTER_THRESHOLD_M
        ):

            live_view_center_x = (
                live_display_robot_x
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
                live_display_robot_y
                -
                live_view_center_y
            )
            >
            LIVE_RECENTER_THRESHOLD_M
        ):

            live_view_center_y = (
                live_display_robot_y
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
        # OCCUPANCY MAP IMAGE
        # ====================================================

        map_image.set_data(
            get_occupancy_display_image()
        )


        # ====================================================
        # MAP ROBOT
        #
        # 0° = +X/right
        # positive navX = clockwise
        # ====================================================

        map_heading_rad = math.radians(
            mapping_robot_heading
        )

        map_heading_dx = (
            0.40
            *
            math.cos(
                map_heading_rad
            )
        )

        map_heading_dy = (
            -0.40
            *
            math.sin(
                map_heading_rad
            )
        )

        map_robot.set_data(
            [mapping_robot_x],
            [mapping_robot_y]
        )

        map_heading_line.set_data(
            [
                mapping_robot_x,
                mapping_robot_x
                +
                map_heading_dx
            ],
            [
                mapping_robot_y,
                mapping_robot_y
                +
                map_heading_dy
            ]
        )

        map_robot_label.set_position(
            (
                mapping_robot_x,
                mapping_robot_y - 0.20
            )
        )

        map_robot_label.set_text(
            (
                "ROBOT\n"
                f"X {mapping_robot_x:.2f}\n"
                f"Y {mapping_robot_y:.2f}\n"
                f"{mapping_robot_heading:.1f}°"
            )
        )


        # ====================================================
        # INFORMATION PANEL
        # ====================================================

        navx_text = (
            "YES"
            if navx_available
            else
            "NO"
        )

        valid_text = (
            "VALID"
            if localization_valid
            else
            "HOLD"
        )

        rotating_text = (
            "YES"
            if pure_rotation_commanded()
            else
            "NO"
        )


        info_text.set_text(
            (
                "FINAL ROBOT POSE               "
                "NAVX HEADING                    "
                "LIDAR TRANSLATION\n"

                f"X: {robot_x:8.3f} m             "
                f"Available: {navx_text:<18} "
                f"State: {localization_state:<22}\n"

                f"Y: {robot_y:8.3f} m             "
                f"Raw yaw: {raw_navx_heading:8.2f}°        "
                f"ICP error: {last_icp_error:7.3f} m\n"

                f"Heading: {robot_heading:8.2f}°           "
                f"Python zero: {navx_zero_offset:8.2f}°    "
                f"Inliers: {last_inlier_ratio * 100:6.1f}%\n"

                f"Valid: {valid_text:<20} "
                f"Frame yaw delta: {last_navx_delta:7.2f}°    "
                f"LiDAR move: {last_relative_translation:7.3f} m\n"

                f"Pose lock: {str(pose_locked):<17} "
                f"Q/E XY hold: {rotating_text:<16} "
                f"Keyframe: {keyframe_number}\n"

                f"Nearest: {obstacle_distance:7.2f} m         "
                f"Rotation holds: {rotation_hold_count:<13} "
                f"Recoveries: {recovery_count}\n"

                f"Logged positions: {len(logged_positions):<11} "
                f"HOLD frames: {consecutive_hold_count:<15} "
                f"Distance scale: {DISTANCE_SCALE_FACTOR:.2f}\n"

                f"Map pose: X={mapping_robot_x:6.2f} "
                f"Y={mapping_robot_y:6.2f} "
                f"H={mapping_robot_heading:6.1f}°     "
                f"Coverage: {get_map_coverage_percent():5.1f}%\n"

                "Direction check: "
                "W = +forward / S = backward\n"

                "\n"

                "Controls:   "
                "R = Reset X/Y/Heading to 0     "
                "L = Log Position     "
                "S = Save Map"
            )
        )


        # ====================================================
        # FIXED MAP SIZE
        #
        # The occupancy map remains centred on the mapping
        # reset origin from -3.5 m to +3.5 m on each axis.
        # ====================================================

        # ====================================================
        # DRAW
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
        "\nLocalization stopped."
    )


finally:

    localization_table.putBoolean(
        "LocalizationValid",
        False
    )

    localization_table.putString(
        "LocalizationState",
        "STOPPED"
    )

    print(
        "Localization closed."
    )