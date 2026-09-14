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
# ============================================================

GRID_SIZE_M = 0.05

OCCUPIED_CONFIRM_THRESHOLD = 3

MAX_CELL_CONFIDENCE = 10

MAP_UPDATE_EVERY_N_FRAMES = 2


# ============================================================
# DISPLAY
# ============================================================

LIVE_POINT_SIZE = 10

MAP_POINT_SIZE = 9

SAVED_MAP_POINT_SIZE = 9

LIVE_HALF_RANGE_M = 5.0

LIVE_RECENTER_THRESHOLD_M = 2.5

MAP_INITIAL_HALF_RANGE_M = 5.0

MAP_EXPAND_MARGIN_M = 1.0

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


# ============================================================
# OCCUPANCY GRID STORAGE
# ============================================================

occupancy_grid = {}


# ============================================================
# DISPLAY LIMITS
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
# RobotPose
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
# PURE Q/E ROTATION?
# ============================================================

def pure_rotation_commanded():

    enabled = keyboard_table.getBoolean(
        "Enabled",
        False
    )

    if not enabled:

        return False

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

    return (
        abs(command_z)
        >=
        KEYBOARD_TURN_THRESHOLD
        and
        abs(command_x)
        <
        KEYBOARD_MOVE_THRESHOLD
        and
        abs(command_y)
        <
        KEYBOARD_MOVE_THRESHOLD
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

    # ========================================================
    # FIX FOR YOUR UnboundLocalError
    # ========================================================

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
    # NAVX IS HEADING AUTHORITY
    # ========================================================

    robot_heading = (
        current_heading
    )


    # ========================================================
    # FIRST SCAN
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

        store_previous_good_scan(
            current_scan
        )

        create_keyframe(
            current_scan
        )

        return


    # ========================================================
    # HEADING CHANGE SINCE PREVIOUS SCAN
    # ========================================================

    navx_delta = heading_difference(
        robot_heading,
        previous_navx_heading
    )

    last_navx_delta = (
        navx_delta
    )


    # ========================================================
    # PURE Q/E ROTATION
    #
    # Heading changes from navX.
    # X/Y stay fixed.
    # ========================================================

    if pure_rotation_commanded():

        localization_valid = True

        localization_state = (
            "NAVX_ROTATING"
        )

        pose_locked = False

        last_relative_translation = (
            0.0
        )

        rotation_hold_count += 1

        consecutive_hold_count = 0


        # ====================================================
        # KEEP SCAN REFERENCE UPDATED DURING ROTATION
        # ====================================================

        previous_good_scan = (
            current_scan.copy()
        )

        previous_good_translation = (
            robot_translation.copy()
        )

        previous_navx_heading = (
            robot_heading
        )


        # ====================================================
        # REFRESH KEYFRAME AFTER ENOUGH ROTATION
        # ====================================================

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
    # NORMAL LIDAR TRANSLATION
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
    # NORMAL SUCCESS
    # ========================================================

    if normal_good:

        relative_distance = float(
            np.linalg.norm(
                relative_translation
            )
        )

        last_relative_translation = (
            relative_distance
        )


        # ====================================================
        # BACKUP ROTATION SUPPRESSION
        # ====================================================

        if (
            abs(navx_delta)
            >=
            ROTATION_SUPPRESSION_MIN_DEG
            and
            relative_distance
            <=
            ROTATION_SUPPRESSION_MAX_TRANSLATION_M
        ):

            relative_translation = np.array(
                [
                    0.0,
                    0.0
                ],
                dtype=float
            )

            localization_state = (
                "ROTATION_HOLD_XY"
            )

            rotation_hold_count += 1

        else:

            localization_state = (
                "TRACKING"
            )


        # ====================================================
        # TRANSLATION -> WORLD
        # ====================================================

        world_delta = local_translation_to_world(
            relative_translation,
            previous_navx_heading
        )


        # ====================================================
        # REMOVE MICRO MOVEMENT
        # ====================================================

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

        consecutive_hold_count = 0


        # ====================================================
        # STORE CURRENT AS NEW REFERENCE
        # ====================================================

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

            recovery_distance = float(
                np.linalg.norm(
                    recovery_translation
                )
            )


            # =================================================
            # ROTATION RECOVERY
            # =================================================

            if (
                abs(
                    keyframe_heading_delta
                )
                >=
                ROTATION_SUPPRESSION_MIN_DEG
                and
                recovery_distance
                <=
                ROTATION_SUPPRESSION_MAX_TRANSLATION_M
            ):

                recovery_translation = np.array(
                    [
                        0.0,
                        0.0
                    ],
                    dtype=float
                )

                localization_state = (
                    "NAVX_ROTATION_RECOVERED"
                )

                rotation_hold_count += 1

            else:

                localization_state = (
                    "RECOVERED"
                )


            world_delta = local_translation_to_world(
                recovery_translation,
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

            pose_locked = False

            recovery_count += 1

            consecutive_hold_count = 0

            last_icp_error = (
                recovery_error
            )

            last_inlier_ratio = (
                recovery_ratio
            )

            last_relative_translation = (
                recovery_distance
            )

            store_previous_good_scan(
                current_scan
            )

            create_keyframe(
                current_scan
            )

            return


    # ========================================================
    # HOLD
    #
    # X/Y stay at the last trusted position.
    #
    # Heading continues following navX.
    # ========================================================

    localization_valid = False

    localization_state = (
        "XY_HOLD_NAVX_OK"
    )

    pose_locked = True

    consecutive_hold_count += 1


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
# OCCUPANCY GRID
# ============================================================

def update_occupancy_grid(
        world_points):

    global occupancy_grid

    if len(world_points) == 0:

        return

    seen = set()

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

        if key in seen:

            continue

        seen.add(
            key
        )

        confidence = occupancy_grid.get(
            key,
            0
        )

        confidence += 1

        confidence = min(
            confidence,
            MAX_CELL_CONFIDENCE
        )

        occupancy_grid[
            key
        ] = confidence


def get_confirmed_map_points():

    points = []

    for (
        key,
        confidence
    ) in occupancy_grid.items():

        if (
            confidence
            >=
            OCCUPIED_CONFIRM_THRESHOLD
        ):

            points.append(
                [
                    key[0]
                    *
                    GRID_SIZE_M,

                    key[1]
                    *
                    GRID_SIZE_M
                ]
            )

    if len(points) == 0:

        return np.empty(
            (0, 2)
        )

    return np.array(
        points,
        dtype=float
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

    if len(points) == 0:

        print(
            "No confirmed map points."
        )

        return

    timestamp = time.strftime(
        "%Y%m%d_%H%M%S"
    )

    csv_path = os.path.join(
        MAP_SAVE_FOLDER,
        "lidar_map_"
        +
        timestamp
        +
        ".csv"
    )

    png_path = os.path.join(
        MAP_SAVE_FOLDER,
        "lidar_map_"
        +
        timestamp
        +
        ".png"
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

    save_fig, save_axis = plt.subplots(
        figsize=(9, 9)
    )

    save_axis.scatter(
        points[:, 0],
        points[:, 1],
        s=SAVED_MAP_POINT_SIZE,
        c="black",
        marker="s"
    )

    for position in logged_positions:

        save_axis.scatter(
            position["x"],
            position["y"],
            s=60,
            marker="o"
        )

        save_axis.text(
            position["x"],
            position["y"] + 0.12,
            (
                f'{position["name"]}\n'
                f'{position["x"]:.2f}, '
                f'{position["y"]:.2f}\n'
                f'{position["heading"]:.1f}°'
            ),
            ha="center"
        )

    save_axis.scatter(
        robot_x,
        robot_y,
        s=100,
        marker="^"
    )

    save_axis.set_aspect(
        "equal",
        adjustable="box"
    )

    save_axis.set_xlabel(
        "World X (m)"
    )

    save_axis.set_ylabel(
        "World Y (m)"
    )

    save_axis.set_title(
        "LiDAR + navX Localization Map"
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
        "MAP SAVED"
    )

    print(
        csv_path
    )

    print(
        png_path
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

    occupancy_grid = {}


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
    "World X (m)"
)

ax_live.set_ylabel(
    "World Y (m)"
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
# ============================================================

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
        # MAP UPDATE
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
        # LIVE ROBOT
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
        # MAP POINTS
        # ====================================================

        if len(
            confirmed_points
        ) > 0:

            map_scatter.set_offsets(
                confirmed_points
            )

        else:

            map_scatter.set_offsets(
                np.empty(
                    (0, 2)
                )
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
                f"Q/E rotating: {rotating_text:<16} "
                f"Keyframe: {keyframe_number}\n"

                f"Nearest: {obstacle_distance:7.2f} m         "
                f"Rotation holds: {rotation_hold_count:<13} "
                f"Recoveries: {recovery_count}\n"

                f"Logged positions: {len(logged_positions):<11} "
                f"HOLD frames: {consecutive_hold_count:<15} "
                f"Distance scale: {DISTANCE_SCALE_FACTOR:.2f}\n"

                "\n"

                "Controls:   "
                "R = Reset X/Y/Heading to 0     "
                "L = Log Position     "
                "S = Save Map"
            )
        )


        # ====================================================
        # MAP AUTO-EXPAND
        # ====================================================

        if len(
            confirmed_points
        ) > 0:

            required_min_x = min(
                float(
                    np.min(
                        confirmed_points[:, 0]
                    )
                ),
                robot_x
            )

            required_max_x = max(
                float(
                    np.max(
                        confirmed_points[:, 0]
                    )
                ),
                robot_x
            )

            required_min_y = min(
                float(
                    np.min(
                        confirmed_points[:, 1]
                    )
                ),
                robot_y
            )

            required_max_y = max(
                float(
                    np.max(
                        confirmed_points[:, 1]
                    )
                ),
                robot_y
            )

            changed = False


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

                changed = True


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

                changed = True


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

                changed = True


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

                changed = True


            if changed:

                ax_map.set_xlim(
                    map_min_x,
                    map_max_x
                )

                ax_map.set_ylim(
                    map_min_y,
                    map_max_y
                )


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