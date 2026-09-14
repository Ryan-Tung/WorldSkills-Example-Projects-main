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
# LIDAR BASIC SETTINGS
# ============================================================

MIN_DISTANCE_MM = 120.0

MAX_DISTANCE_MM = 5000.0

MIN_SCAN_POINTS = 30


# ============================================================
# LIDAR MOUNTING ANGLE
#
# This belongs to the robot itself.
# ============================================================

LIDAR_MOUNT_OFFSET_DEG = -20.0


# ============================================================
# DISTANCE SCALE
#
# Flexible arena.
#
# No known wall is required.
# ============================================================

DISTANCE_SCALE_FACTOR = 1.0


# ============================================================
# LIDAR OFFSET FROM ROBOT ROTATION CENTRE
#
# +X = right
# +Y = forward
#
# If LiDAR is approximately at robot centre:
#
# leave both as 0.
# ============================================================

LIDAR_OFFSET_X_M = 0.0

LIDAR_OFFSET_Y_M = 0.0


# ============================================================
# HEADING STABILIZATION
#
# Previous:
#
# 0.35 caused quite noticeable lag.
#
# 0.75 follows the raw LiDAR heading much faster.
# ============================================================

HEADING_SMOOTHING_ALPHA = 0.75


# Only clamp clearly unrealistic frame-to-frame jumps.
MAX_HEADING_CHANGE_PER_FRAME_DEG = 25.0


# Ignore only very tiny noise.
MIN_HEADING_CHANGE_TO_UPDATE_DEG = 0.10


stable_robot_heading = 0.0

stable_heading_initialized = False


# ============================================================
# NORMAL ICP
#
# Current scan -> previous successful scan.
#
# This is the main localization method.
# ============================================================

NORMAL_ICP_POINTS = 220

NORMAL_ICP_ITERATIONS = 8

NORMAL_CORRESPONDENCE_M = 0.50

NORMAL_MIN_MATCHES = 18


NORMAL_MAX_ERROR_M = 0.10

NORMAL_MIN_INLIER_RATIO = 0.28

NORMAL_MAX_TRANSLATION_M = 0.60

NORMAL_MAX_ROTATION_DEG = 30.0


# ============================================================
# RECOVERY ICP
#
# More tolerant than normal tracking.
# ============================================================

RECOVERY_ICP_POINTS = 300

RECOVERY_ICP_ITERATIONS = 12

RECOVERY_CORRESPONDENCE_M = 0.75

RECOVERY_MIN_MATCHES = 15


RECOVERY_MAX_ERROR_M = 0.16

RECOVERY_MIN_INLIER_RATIO = 0.20

RECOVERY_MAX_TRANSLATION_M = 1.20

RECOVERY_MAX_ROTATION_DEG = 60.0


# ============================================================
# ROTATION-IN-PLACE FILTER
#
# This prevents Q / E from drawing a large fake circle.
# ============================================================

ROTATION_ONLY_MIN_ANGLE_DEG = 0.70

ROTATION_ONLY_MAX_TRANSLATION_M = 0.10


STRONG_ROTATION_MIN_ANGLE_DEG = 2.00

STRONG_ROTATION_MAX_TRANSLATION_M = 0.16


# ============================================================
# KEYFRAME SETTINGS
# ============================================================

KEYFRAME_TRANSLATION_M = 0.18

KEYFRAME_ROTATION_DEG = 5.0


# ============================================================
# KEYFRAME CORRECTION ACCEPTANCE
# ============================================================

KEYFRAME_CORRECTION_MAX_POSITION_DIFF_M = 0.25

KEYFRAME_CORRECTION_MAX_HEADING_DIFF_DEG = 10.0


# ============================================================
# IMPORTANT:
#
# DON'T keep "correcting" the keyframe while robot is sitting
# still.
#
# Your screenshot showed:
#
# Corrections: 166
#
# while basically stationary.
#
# These thresholds stop that.
# ============================================================

MIN_KEYFRAME_CORRECTION_TRANSLATION_M = 0.03

MIN_KEYFRAME_CORRECTION_ROTATION_DEG = 1.50


# ============================================================
# STATIONARY LOCK
# ============================================================

STATIONARY_TRANSLATION_M = 0.008

STATIONARY_ROTATION_DEG = 0.25


# ============================================================
# LIVE POINT FILTER
# ============================================================

LIVE_CLUSTER_GAP_M = 0.20

LIVE_MIN_CLUSTER_POINTS = 4


# ============================================================
# OCCUPANCY MAP
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
# RAW LIDAR SENSOR POSE
#
# Used internally by ICP.
# ============================================================

lidar_x = 0.0

lidar_y = 0.0

lidar_heading = 0.0


lidar_rotation = np.eye(2)


lidar_translation = np.array(
    [
        0.0,
        0.0
    ],
    dtype=float
)


# ============================================================
# STABLE ROBOT CENTRE POSE
#
# These values are sent to keyboard_drive.py.
# ============================================================

robot_x = 0.0

robot_y = 0.0

robot_heading = 0.0


# ============================================================
# PREVIOUS SUCCESSFUL SCAN
# ============================================================

previous_good_scan = None


previous_good_rotation = np.eye(2)


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


keyframe_rotation = np.eye(2)


keyframe_translation = np.array(
    [
        0.0,
        0.0
    ],
    dtype=float
)


keyframe_number = 0


# ============================================================
# LOCALIZATION STATUS
# ============================================================

localization_valid = False

localization_state = "STARTING"

pose_locked = False


recovery_count = 0

keyframe_correction_count = 0

rotation_filter_count = 0

consecutive_hold_count = 0


frame_counter = 0


last_icp_error = 999.0

last_inlier_ratio = 0.0


last_relative_translation = 0.0

last_relative_rotation = 0.0

last_rotation_filter_active = False


# ============================================================
# MAP STORAGE
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

    return abs(
        normalize_heading(
            first
            -
            second
        )
    )


# ============================================================
# STABLE HEADING FILTER
#
# Uses shortest angular difference.
#
# Example:
#
# 179° -> -179°
#
# is treated as 2°, not 358°.
# ============================================================

def update_stable_heading(
        raw_heading):

    global stable_robot_heading

    global stable_heading_initialized


    raw_heading = normalize_heading(
        raw_heading
    )


    # ========================================================
    # FIRST VALUE
    # ========================================================

    if not stable_heading_initialized:

        stable_robot_heading = (
            raw_heading
        )


        stable_heading_initialized = True


        return (
            stable_robot_heading
        )


    # ========================================================
    # SHORTEST DIFFERENCE
    # ========================================================

    difference = normalize_heading(
        raw_heading
        -
        stable_robot_heading
    )


    # ========================================================
    # IGNORE VERY SMALL JITTER
    # ========================================================

    if (
        abs(
            difference
        )
        <
        MIN_HEADING_CHANGE_TO_UPDATE_DEG
    ):

        return (
            stable_robot_heading
        )


    # ========================================================
    # LIMIT ONLY BIG BAD JUMPS
    # ========================================================

    if (
        abs(
            difference
        )
        >
        MAX_HEADING_CHANGE_PER_FRAME_DEG
    ):

        difference = math.copysign(
            MAX_HEADING_CHANGE_PER_FRAME_DEG,
            difference
        )


    # ========================================================
    # RESPONSIVE SMOOTHING
    # ========================================================

    stable_robot_heading = normalize_heading(
        stable_robot_heading
        +
        HEADING_SMOOTHING_ALPHA
        *
        difference
    )


    return (
        stable_robot_heading
    )


# ============================================================
# ROTATION MATRIX -> HEADING
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
# UPDATE ROBOT CENTRE FROM LIDAR SENSOR POSE
# ============================================================

def update_robot_pose_from_lidar():

    global robot_x

    global robot_y

    global robot_heading


    # ========================================================
    # STABLE USER-VISIBLE HEADING
    # ========================================================

    robot_heading = update_stable_heading(
        lidar_heading
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


    # ========================================================
    # SENSOR OFFSET IN WORLD FRAME
    # ========================================================

    offset_world_x = (
        LIDAR_OFFSET_X_M
        *
        cos_h
        +
        LIDAR_OFFSET_Y_M
        *
        sin_h
    )


    offset_world_y = (
        -LIDAR_OFFSET_X_M
        *
        sin_h
        +
        LIDAR_OFFSET_Y_M
        *
        cos_h
    )


    # ========================================================
    # ROBOT ROTATION CENTRE
    # ========================================================

    robot_x = (
        lidar_x
        -
        offset_world_x
    )


    robot_y = (
        lidar_y
        -
        offset_world_y
    )


# ============================================================
# PARSE LIDAR SCAN
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


            corrected_distance_mm = (
                raw_distance_mm
                *
                DISTANCE_SCALE_FACTOR
            )


            angles.append(
                corrected_angle
            )


            distances.append(
                corrected_distance_mm
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
# POLAR -> ROBOT LOCAL XY
#
# 0°   = +Y = forward
# 90°  = +X = right
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


    x = (
        distance_m
        *
        np.sin(
            angle_rad
        )
    )


    y = (
        distance_m
        *
        np.cos(
            angle_rad
        )
    )


    return np.column_stack(
        (
            x,
            y
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
# BEST-FIT RIGID TRANSFORM
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


    h_matrix = (
        source_zero.T
        @
        target_zero
    )


    u_matrix, _, vt_matrix = np.linalg.svd(
        h_matrix
    )


    rotation = (
        vt_matrix.T
        @
        u_matrix.T
    )


    if np.linalg.det(
        rotation
    ) < 0:

        vt_matrix[-1, :] *= -1


        rotation = (
            vt_matrix.T
            @
            u_matrix.T
        )


    translation = (
        target_center
        -
        rotation
        @
        source_center
    )


    return (
        rotation,
        translation
    )


# ============================================================
# GENERIC ICP
# ============================================================

def run_icp(
        current_points,
        reference_points,
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
            np.eye(2),
            np.zeros(2),
            False,
            999.0,
            0.0
        )


    transformed = (
        current.copy()
    )


    total_rotation = np.eye(2)


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
                np.eye(2),
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


        (
            step_rotation,
            step_translation
        ) = best_fit_transform(
            source_match,
            target_match
        )


        transformed = (
            transformed
            @
            step_rotation.T
        ) + step_translation


        total_translation = (
            step_rotation
            @
            total_translation
            +
            step_translation
        )


        total_rotation = (
            step_rotation
            @
            total_rotation
        )


        error = float(
            np.mean(
                distances[
                    valid
                ]
            )
        )


        if previous_error is not None:

            improvement = abs(
                previous_error
                -
                error
            )


            if improvement < 0.00015:

                break


        previous_error = (
            error
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
        total_rotation,
        total_translation,
        True,
        previous_error,
        final_ratio
    )


# ============================================================
# NORMAL ICP
# ============================================================

def normal_icp(
        current,
        reference):

    return run_icp(
        current,
        reference,

        NORMAL_ICP_POINTS,

        NORMAL_ICP_ITERATIONS,

        NORMAL_CORRESPONDENCE_M,

        NORMAL_MIN_MATCHES
    )


# ============================================================
# RECOVERY ICP
# ============================================================

def recovery_icp(
        current,
        reference):

    return run_icp(
        current,
        reference,

        RECOVERY_ICP_POINTS,

        RECOVERY_ICP_ITERATIONS,

        RECOVERY_CORRESPONDENCE_M,

        RECOVERY_MIN_MATCHES
    )


# ============================================================
# QUALITY CHECK
# ============================================================

def transform_quality_good(
        rotation,
        translation,
        error,
        ratio,
        recovery=False):

    movement = float(
        np.linalg.norm(
            translation
        )
    )


    rotation_amount = abs(
        get_heading_from_rotation(
            rotation
        )
    )


    if recovery:

        return (
            error
            <=
            RECOVERY_MAX_ERROR_M
            and
            ratio
            >=
            RECOVERY_MIN_INLIER_RATIO
            and
            movement
            <=
            RECOVERY_MAX_TRANSLATION_M
            and
            rotation_amount
            <=
            RECOVERY_MAX_ROTATION_DEG
        )


    return (
        error
        <=
        NORMAL_MAX_ERROR_M
        and
        ratio
        >=
        NORMAL_MIN_INLIER_RATIO
        and
        movement
        <=
        NORMAL_MAX_TRANSLATION_M
        and
        rotation_amount
        <=
        NORMAL_MAX_ROTATION_DEG
    )


# ============================================================
# ROTATION-DOMINANT MOTION?
# ============================================================

def is_rotation_dominant(
        rotation,
        translation):

    rotation_amount = abs(
        get_heading_from_rotation(
            rotation
        )
    )


    movement = float(
        np.linalg.norm(
            translation
        )
    )


    if (
        rotation_amount
        >=
        STRONG_ROTATION_MIN_ANGLE_DEG
        and
        movement
        <=
        STRONG_ROTATION_MAX_TRANSLATION_M
    ):

        return True


    if (
        rotation_amount
        >=
        ROTATION_ONLY_MIN_ANGLE_DEG
        and
        movement
        <=
        ROTATION_ONLY_MAX_TRANSLATION_M
    ):

        return True


    return False


# ============================================================
# REMOVE FALSE TRANSLATION DURING ROTATION
# ============================================================

def filter_relative_motion(
        rotation,
        translation):

    global rotation_filter_count

    global last_relative_translation

    global last_relative_rotation

    global last_rotation_filter_active


    movement = float(
        np.linalg.norm(
            translation
        )
    )


    rotation_amount = abs(
        get_heading_from_rotation(
            rotation
        )
    )


    last_relative_translation = (
        movement
    )


    last_relative_rotation = (
        rotation_amount
    )


    if is_rotation_dominant(
        rotation,
        translation
    ):

        rotation_filter_count += 1


        last_rotation_filter_active = True


        return (
            rotation,

            np.array(
                [
                    0.0,
                    0.0
                ],
                dtype=float
            ),

            True
        )


    last_rotation_filter_active = False


    return (
        rotation,
        translation.copy(),
        False
    )


# ============================================================
# REFERENCE -> GLOBAL LIDAR POSE
# ============================================================

def calculate_pose_from_reference(
        reference_rotation,
        reference_translation,
        relative_rotation,
        relative_translation):

    new_rotation = (
        reference_rotation
        @
        relative_rotation
    )


    new_translation = (
        reference_translation
        +
        reference_rotation
        @
        relative_translation
    )


    return (
        new_rotation,
        new_translation
    )


# ============================================================
# SET LIDAR POSE
# ============================================================

def set_lidar_pose(
        rotation,
        translation,
        allow_stationary_lock=True):

    global lidar_rotation

    global lidar_translation

    global lidar_x

    global lidar_y

    global lidar_heading

    global pose_locked


    proposed_x = float(
        translation[0]
    )


    proposed_y = float(
        translation[1]
    )


    proposed_heading = (
        get_heading_from_rotation(
            rotation
        )
    )


    movement = math.sqrt(
        (
            proposed_x
            -
            lidar_x
        ) ** 2
        +
        (
            proposed_y
            -
            lidar_y
        ) ** 2
    )


    heading_change = (
        heading_difference(
            proposed_heading,
            lidar_heading
        )
    )


    # ========================================================
    # STATIONARY LOCK
    # ========================================================

    if (
        allow_stationary_lock
        and
        movement
        <
        STATIONARY_TRANSLATION_M
        and
        heading_change
        <
        STATIONARY_ROTATION_DEG
    ):

        pose_locked = True


        update_robot_pose_from_lidar()


        return


    # ========================================================
    # ACCEPT POSE
    # ========================================================

    lidar_rotation = (
        rotation.copy()
    )


    lidar_translation = (
        translation.copy()
    )


    lidar_x = proposed_x

    lidar_y = proposed_y

    lidar_heading = (
        proposed_heading
    )


    pose_locked = False


    update_robot_pose_from_lidar()


# ============================================================
# STORE PREVIOUS GOOD SCAN
# ============================================================

def store_previous_good_scan(
        scan):

    global previous_good_scan

    global previous_good_rotation

    global previous_good_translation


    previous_good_scan = (
        scan.copy()
    )


    previous_good_rotation = (
        lidar_rotation.copy()
    )


    previous_good_translation = (
        lidar_translation.copy()
    )


# ============================================================
# CREATE KEYFRAME
# ============================================================

def create_keyframe(
        scan):

    global keyframe_scan

    global keyframe_rotation

    global keyframe_translation

    global keyframe_number


    keyframe_scan = (
        scan.copy()
    )


    keyframe_rotation = (
        lidar_rotation.copy()
    )


    keyframe_translation = (
        lidar_translation.copy()
    )


    keyframe_number += 1


# ============================================================
# KEYFRAME NEEDED?
# ============================================================

def keyframe_needed():

    movement = math.sqrt(
        (
            lidar_x
            -
            keyframe_translation[0]
        ) ** 2
        +
        (
            lidar_y
            -
            keyframe_translation[1]
        ) ** 2
    )


    key_heading = (
        get_heading_from_rotation(
            keyframe_rotation
        )
    )


    heading_change = (
        heading_difference(
            lidar_heading,
            key_heading
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
# SHOULD WE EVEN TRY KEYFRAME CORRECTION?
#
# Prevent correction counter from going crazy when stationary.
# ============================================================

def meaningful_motion_for_keyframe_correction(
        relative_rotation,
        relative_translation):

    movement = float(
        np.linalg.norm(
            relative_translation
        )
    )


    rotation_amount = abs(
        get_heading_from_rotation(
            relative_rotation
        )
    )


    return (
        movement
        >=
        MIN_KEYFRAME_CORRECTION_TRANSLATION_M
        or
        rotation_amount
        >=
        MIN_KEYFRAME_CORRECTION_ROTATION_DEG
    )


# ============================================================
# LOCALIZATION
# ============================================================

def track_localization(
        current_scan):

    global localization_valid

    global localization_state

    global recovery_count

    global keyframe_correction_count

    global rotation_filter_count

    global consecutive_hold_count

    global last_icp_error

    global last_inlier_ratio

    global pose_locked


    # ========================================================
    # FIRST SCAN
    # ========================================================

    if previous_good_scan is None:

        set_lidar_pose(
            np.eye(2),

            np.array(
                [
                    0.0,
                    0.0
                ],
                dtype=float
            ),

            False
        )


        localization_valid = True


        localization_state = (
            "INITIALIZED"
        )


        consecutive_hold_count = 0


        store_previous_good_scan(
            current_scan
        )


        create_keyframe(
            current_scan
        )


        return


    # ========================================================
    # PRIMARY:
    #
    # CURRENT SCAN -> PREVIOUS GOOD SCAN
    # ========================================================

    (
        prev_rotation,
        prev_translation,
        prev_success,
        prev_error,
        prev_ratio
    ) = normal_icp(
        current_scan,
        previous_good_scan
    )


    previous_match_good = (
        prev_success
        and
        transform_quality_good(
            prev_rotation,
            prev_translation,
            prev_error,
            prev_ratio,
            False
        )
    )


    # ========================================================
    # PRIMARY SUCCESS
    # ========================================================

    if previous_match_good:

        (
            filtered_rotation,
            filtered_translation,
            rotation_filtered
        ) = filter_relative_motion(
            prev_rotation,
            prev_translation
        )


        (
            candidate_rotation,
            candidate_translation
        ) = calculate_pose_from_reference(
            previous_good_rotation,
            previous_good_translation,

            filtered_rotation,
            filtered_translation
        )


        last_icp_error = (
            prev_error
        )


        last_inlier_ratio = (
            prev_ratio
        )


        if rotation_filtered:

            localization_state = (
                "ROTATING"
            )


        else:

            localization_state = (
                "TRACKING"
            )


        # ====================================================
        # ONLY TRY KEYFRAME CORRECTION IF THERE WAS
        # MEANINGFUL MOTION
        # ====================================================

        allow_keyframe_correction = (
            meaningful_motion_for_keyframe_correction(
                prev_rotation,
                prev_translation
            )
        )


        if (
            keyframe_scan is not None
            and
            allow_keyframe_correction
        ):

            (
                key_rotation,
                key_translation,
                key_success,
                key_error,
                key_ratio
            ) = normal_icp(
                current_scan,
                keyframe_scan
            )


            key_match_good = (
                key_success
                and
                transform_quality_good(
                    key_rotation,
                    key_translation,
                    key_error,
                    key_ratio,
                    False
                )
            )


            if key_match_good:

                (
                    corrected_rotation,
                    corrected_translation
                ) = calculate_pose_from_reference(
                    keyframe_rotation,
                    keyframe_translation,

                    key_rotation,
                    key_translation
                )


                candidate_heading = (
                    get_heading_from_rotation(
                        candidate_rotation
                    )
                )


                corrected_heading = (
                    get_heading_from_rotation(
                        corrected_rotation
                    )
                )


                position_difference = float(
                    np.linalg.norm(
                        corrected_translation
                        -
                        candidate_translation
                    )
                )


                heading_diff = (
                    heading_difference(
                        corrected_heading,
                        candidate_heading
                    )
                )


                # ============================================
                # ROTATION:
                #
                # correct heading only
                #
                # NEVER let the keyframe move X/Y.
                # ============================================

                if rotation_filtered:

                    if (
                        heading_diff
                        <=
                        KEYFRAME_CORRECTION_MAX_HEADING_DIFF_DEG
                    ):

                        candidate_rotation = (
                            corrected_rotation
                        )


                        localization_state = (
                            "ROTATION_CORRECTED"
                        )


                        keyframe_correction_count += 1


                        last_icp_error = (
                            key_error
                        )


                        last_inlier_ratio = (
                            key_ratio
                        )


                # ============================================
                # TRANSLATION:
                #
                # full correction only when both agree.
                # ============================================

                else:

                    if (
                        position_difference
                        <=
                        KEYFRAME_CORRECTION_MAX_POSITION_DIFF_M
                        and
                        heading_diff
                        <=
                        KEYFRAME_CORRECTION_MAX_HEADING_DIFF_DEG
                    ):

                        candidate_rotation = (
                            corrected_rotation
                        )


                        candidate_translation = (
                            corrected_translation
                        )


                        localization_state = (
                            "KEYFRAME_CORRECTED"
                        )


                        keyframe_correction_count += 1


                        last_icp_error = (
                            key_error
                        )


                        last_inlier_ratio = (
                            key_ratio
                        )


        # ====================================================
        # APPLY TRUSTED POSE
        # ====================================================

        set_lidar_pose(
            candidate_rotation,
            candidate_translation,
            True
        )


        localization_valid = True

        consecutive_hold_count = 0


        # ====================================================
        # CURRENT SCAN BECOMES NEW PREVIOUS GOOD SCAN
        # ====================================================

        store_previous_good_scan(
            current_scan
        )


        # ====================================================
        # KEYFRAME UPDATE
        # ====================================================

        if keyframe_needed():

            create_keyframe(
                current_scan
            )


        return


    # ========================================================
    # PRIMARY FAILED
    #
    # TRY RECOVERY FROM KEYFRAME
    # ========================================================

    if keyframe_scan is not None:

        (
            recovery_rotation,
            recovery_translation,
            recovery_success,
            recovery_error,
            recovery_ratio
        ) = recovery_icp(
            current_scan,
            keyframe_scan
        )


        recovery_good = (
            recovery_success
            and
            transform_quality_good(
                recovery_rotation,
                recovery_translation,
                recovery_error,
                recovery_ratio,
                True
            )
        )


        if recovery_good:

            (
                recovered_rotation,
                recovered_translation
            ) = calculate_pose_from_reference(
                keyframe_rotation,
                keyframe_translation,

                recovery_rotation,
                recovery_translation
            )


            # =================================================
            # ROTATION RECOVERY
            #
            # HOLD X/Y.
            # =================================================

            if is_rotation_dominant(
                recovery_rotation,
                recovery_translation
            ):

                recovered_translation = (
                    lidar_translation.copy()
                )


                localization_state = (
                    "ROTATION_RECOVERED"
                )


                rotation_filter_count += 1


            else:

                localization_state = (
                    "RECOVERED"
                )


            set_lidar_pose(
                recovered_rotation,
                recovered_translation,
                False
            )


            localization_valid = True


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
    # HOLD
    #
    # X / Y / Heading remain exactly where last trusted.
    # ========================================================

    localization_valid = False

    localization_state = "HOLD"

    pose_locked = True


    consecutive_hold_count += 1


# ============================================================
# LOCAL LIDAR POINTS -> WORLD
#
# Map uses RAW LiDAR pose internally.
# ============================================================

def local_to_world(
        local_points):

    if len(local_points) == 0:

        return np.empty(
            (0, 2)
        )


    heading_rad = math.radians(
        lidar_heading
    )


    cos_h = math.cos(
        heading_rad
    )


    sin_h = math.sin(
        heading_rad
    )


    world_x = (
        lidar_x
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
        lidar_y
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
# LIVE CLUSTERS
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


        if gap <= LIVE_CLUSTER_GAP_M:

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


    local = local_points[
        index
    ]


    world = world_points[
        index
    ]


    angle = math.degrees(
        math.atan2(
            local[0],
            local[1]
        )
    )


    return (
        float(
            world[0]
        ),

        float(
            world[1]
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


    if not localization_valid:

        last_log_message = (
            "LOG FAILED - LOCALIZATION HOLD"
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

    # ========================================================
    # STABLE ROBOT POSE
    # ========================================================

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


    # ========================================================
    # RAW INTERNAL LIDAR POSE
    # ========================================================

    localization_table.putNumber(
        "LidarX",
        lidar_x
    )


    localization_table.putNumber(
        "LidarY",
        lidar_y
    )


    localization_table.putNumber(
        "LidarHeading",
        lidar_heading
    )


    # ========================================================
    # OBSTACLE
    # ========================================================

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


    # ========================================================
    # STATUS
    # ========================================================

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
        "RotationFilterCount",
        rotation_filter_count
    )


    localization_table.putNumber(
        "KeyframeCorrectionCount",
        keyframe_correction_count
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


    # ========================================================
    # CSV
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


        for point in points:

            writer.writerow(
                [
                    point[0],
                    point[1]
                ]
            )


    # ========================================================
    # PNG
    # ========================================================

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
        "LiDAR Confirmed Map"
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
        "MAP SAVED:"
    )


    print(
        csv_path
    )


    print(
        png_path
    )


# ============================================================
# RESET
# ============================================================

def reset_localization():

    global lidar_x

    global lidar_y

    global lidar_heading

    global lidar_rotation

    global lidar_translation


    global robot_x

    global robot_y

    global robot_heading


    global stable_robot_heading

    global stable_heading_initialized


    global previous_good_scan

    global previous_good_rotation

    global previous_good_translation


    global keyframe_scan

    global keyframe_rotation

    global keyframe_translation

    global keyframe_number


    global localization_valid

    global localization_state

    global pose_locked


    global recovery_count

    global keyframe_correction_count

    global rotation_filter_count

    global consecutive_hold_count


    global frame_counter


    global last_icp_error

    global last_inlier_ratio


    global last_relative_translation

    global last_relative_rotation

    global last_rotation_filter_active


    global occupancy_grid


    # ========================================================
    # LIDAR
    # ========================================================

    lidar_x = 0.0

    lidar_y = 0.0

    lidar_heading = 0.0


    lidar_rotation = np.eye(2)


    lidar_translation = np.array(
        [
            0.0,
            0.0
        ],
        dtype=float
    )


    # ========================================================
    # ROBOT
    # ========================================================

    robot_x = 0.0

    robot_y = 0.0

    robot_heading = 0.0


    # ========================================================
    # HEADING FILTER
    # ========================================================

    stable_robot_heading = 0.0

    stable_heading_initialized = False


    # ========================================================
    # PREVIOUS SCAN
    # ========================================================

    previous_good_scan = None


    previous_good_rotation = np.eye(2)


    previous_good_translation = np.array(
        [
            0.0,
            0.0
        ],
        dtype=float
    )


    # ========================================================
    # KEYFRAME
    # ========================================================

    keyframe_scan = None


    keyframe_rotation = np.eye(2)


    keyframe_translation = np.array(
        [
            0.0,
            0.0
        ],
        dtype=float
    )


    keyframe_number = 0


    # ========================================================
    # STATUS
    # ========================================================

    localization_valid = False

    localization_state = (
        "STARTING"
    )


    pose_locked = False


    recovery_count = 0

    keyframe_correction_count = 0

    rotation_filter_count = 0

    consecutive_hold_count = 0


    frame_counter = 0


    last_icp_error = 999.0

    last_inlier_ratio = 0.0


    last_relative_translation = 0.0

    last_relative_rotation = 0.0

    last_rotation_filter_active = False


    occupancy_grid = {}


    print(
        "Localization reset."
    )


# ============================================================
# KEYBOARD
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
# Live map | Persistent map
#
# BOTTOM:
#
# Information panel spanning full width.
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
        1.25
    ],
    hspace=0.22,
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
    "LiDAR Localization - Stable Pose + Yaw",
    fontsize=16
)


plt.show(
    block=False
)


# ============================================================
# DON'T FORCE WINDOW ALWAYS ON TOP
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
# BOTTOM INFORMATION PANEL
# ============================================================

ax_info.set_axis_off()


info_text = ax_info.text(
    0.01,
    0.92,
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
# START INFO
# ============================================================

print(
    "=========================================="
)


print(
    "LIDAR LOCALIZATION"
)


print(
    "STABLE POSE + RESPONSIVE YAW"
)


print(
    "=========================================="
)


print(
    "No reference wall required."
)


print(
    "First scan = X0 Y0 Heading0."
)


print(
    "Heading smoothing alpha:",
    HEADING_SMOOTHING_ALPHA
)


print(
    "Keyframe idle correction protection enabled."
)


print(
    "Rotation translation suppression enabled."
)


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
        # POINT CLOUD
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
        # OBJECT FILTERING
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
        # NETWORKTABLE
        # ====================================================

        publish_localization(
            obstacle_x,
            obstacle_y,
            obstacle_distance,
            obstacle_angle
        )


        # ====================================================
        # STABLE HEADING ARROW
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
        # LIVE ENVIRONMENT
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
        # CONFIRMED MAP
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
        # BOTTOM INFORMATION PANEL
        # ====================================================

        valid_text = (
            "VALID"
            if localization_valid
            else
            "HOLD"
        )


        info_text.set_text(
            (
                "ROBOT POSE                    "
                "LOCALIZATION                   "
                "ICP / MOTION\n"

                f"X: {robot_x:7.3f} m              "
                f"State: {localization_state:<20} "
                f"ICP error: {last_icp_error:7.3f} m\n"

                f"Y: {robot_y:7.3f} m              "
                f"Valid: {valid_text:<20} "
                f"Inliers: {last_inlier_ratio * 100:6.1f}%\n"

                f"Stable heading: {robot_heading:7.2f}°     "
                f"Keyframe: {keyframe_number:<17} "
                f"Relative move: {last_relative_translation:6.3f} m\n"

                f"Raw heading:    {lidar_heading:7.2f}°     "
                f"Corrections: {keyframe_correction_count:<16} "
                f"Relative rotation: {last_relative_rotation:6.2f}°\n"

                f"Pose lock: {str(pose_locked):<10}          "
                f"Recoveries: {recovery_count:<17} "
                f"Rotation filter: "
                f"{'ON' if last_rotation_filter_active else 'OFF'}\n"

                f"Nearest object: {obstacle_distance:6.2f} m       "
                f"HOLD frames: {consecutive_hold_count:<16} "
                f"Rotation filters: {rotation_filter_count}\n"

                "\n"

                "Controls:  "
                "L = Log position    "
                "S = Save map    "
                "R = Reset localization"
            )
        )


        # ====================================================
        # MAP RANGE
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
        "\nLiDAR localization stopped."
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