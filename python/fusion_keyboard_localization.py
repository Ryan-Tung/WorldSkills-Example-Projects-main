import os
import csv
import time
import math
import tkinter as tk
from tkinter import ttk

import numpy as np

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import FancyArrowPatch

from networktables import NetworkTables
from scipy.spatial import cKDTree


# ============================================================
# ROBOT / NETWORKTABLES
# ============================================================

ROBOT_IP = "10.23.45.2"

NetworkTables.initialize(
    server=ROBOT_IP
)

NetworkTables.setUpdateRate(
    0.010
)

keyboard_table = NetworkTables.getTable(
    "KeyboardDrive"
)

lidar_table = NetworkTables.getTable(
    "Lidar"
)

robot_pose_table = NetworkTables.getTable(
    "RobotPose"
)

lidar_localization_table = NetworkTables.getTable(
    "LidarLocalization"
)

fusion_table = NetworkTables.getTable(
    "FusionLocalization"
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
# KEYBOARD DRIVE SETTINGS
# ============================================================

MOVE_SPEED = 0.25
TURN_SPEED = 0.15

DRIVE_UPDATE_MS = 50
DISPLAY_UPDATE_MS = 100

COMMAND_MOVE_THRESHOLD = 0.05
COMMAND_TURN_THRESHOLD = 0.05


# ============================================================
# LIDAR SETTINGS
# ============================================================

MIN_DISTANCE_MM = 120.0
MAX_DISTANCE_MM = 5000.0
MIN_SCAN_POINTS = 30

LIDAR_MOUNT_OFFSET_DEG = -20.0
DISTANCE_SCALE_FACTOR = 1.0

LIVE_POINT_SIZE = 8
LIVE_HALF_RANGE_M = 5.0


# ============================================================
# NAVX / LIDAR ICP SETTINGS
#
# LiDAR ICP publishes LidarLocalization/RobotX / RobotY.
# Java FusionLocalization uses those values as correction data.
#
# navX supplies heading.
# ============================================================

NAVX_HEADING_SIGN = 1.0

NORMAL_ICP_POINTS = 260
NORMAL_ICP_ITERATIONS = 8
NORMAL_CORRESPONDENCE_M = 0.45
NORMAL_MIN_MATCHES = 20
NORMAL_MAX_ERROR_M = 0.10
NORMAL_MIN_INLIER_RATIO = 0.30
NORMAL_MAX_TRANSLATION_M = 0.45

RECOVERY_ICP_POINTS = 320
RECOVERY_ICP_ITERATIONS = 12
RECOVERY_CORRESPONDENCE_M = 0.70
RECOVERY_MIN_MATCHES = 15
RECOVERY_MAX_ERROR_M = 0.16
RECOVERY_MIN_INLIER_RATIO = 0.20
RECOVERY_MAX_TRANSLATION_M = 1.00

KEYFRAME_TRANSLATION_M = 0.20
KEYFRAME_ROTATION_DEG = 8.0

STATIONARY_TRANSLATION_M = 0.006


# ============================================================
# OCCUPANCY MAP
#
# Fixed 7 m x 7 m map.
# 5 cm per cell = 140 x 140 cells.
#
# Evidence:
#   negative = free
#   positive = occupied
#   zero     = unknown
# ============================================================

MAP_SIZE_M = 7.0
GRID_RESOLUTION_M = 0.05
GRID_CELLS = int(
    round(
        MAP_SIZE_M
        /
        GRID_RESOLUTION_M
    )
)

FREE_UPDATE = -1
OCCUPIED_UPDATE = 3

MIN_GRID_EVIDENCE = -8
MAX_GRID_EVIDENCE = 12

FREE_THRESHOLD = -2
OCCUPIED_THRESHOLD = 3

MAP_UPDATE_EVERY_N_FRAMES = 2

# Pause persistent-map writes while fusion is making a large
# correction. The LIVE LiDAR panel still updates continuously.
# This prevents a collision/wheel-slip event from smearing walls
# across the persistent occupancy map while the fused pose recovers.
MAP_MAX_FUSION_ERROR_M = 0.15
MAP_MAX_CORRECTION_APPLIED_M = 0.04


# ============================================================
# ROUTE / DISPLAY
#
# Display convention:
#
#                  -90 / 270°
#                      ↑
#                      |
# 180°  <--------------+--------------> 0° / +X
#                      |
#                      ↓
#                     90°
#
# navX positive direction is clockwise.
# ============================================================

HEADING_ARROW_LENGTH_M = 0.30
ROUTE_MAX_POINTS = 5000
MAX_VISIBLE_LOG_ROWS = 15


# ============================================================
# PROGRAM STATE
# ============================================================

program_running = True
pressed_keys = set()
space_stop_active = False
heartbeat = 0

current_command_x = 0.0
current_command_y = 0.0
current_command_z = 0.0


# ============================================================
# RAW FUSION POSE
# ============================================================

raw_fused_x = 0.0
raw_fused_y = 0.0
raw_fused_heading = 0.0
fusion_active = False

# Full fusion reset handshake.
#
# C now resets the ACTUAL Java fusion frame instead of only
# rotating/re-zeroing the Python display. This keeps:
#
# - encoder odometry
# - navX heading
# - LiDAR ICP
# - fused route
# - occupancy map
#
# on one synchronized zero reference.
fusion_reset_request_id = int(
    fusion_table.getNumber(
        "ResetRequestId",
        0.0
    )
)
fusion_reset_pending = False
fusion_reset_requested_at = 0.0
FUSION_RESET_TIMEOUT_S = 3.0

# Kept for compatibility with the previous code structure.
# The corrected display no longer performs a second arbitrary
# heading rotation here; the Java fusion frame itself is reset.
fusion_zero_initialized = False
fusion_zero_x = 0.0
fusion_zero_y = 0.0
fusion_zero_heading = 0.0

display_robot_x = 0.0
display_robot_y = 0.0
display_robot_heading = 0.0


# ============================================================
# LIDAR LOCALIZATION STATE
# ============================================================

navx_zero_offset = 0.0
navx_zero_initialized = False

lidar_robot_x = 0.0
lidar_robot_y = 0.0
lidar_robot_heading = 0.0

previous_navx_heading = 0.0

previous_good_scan = None
keyframe_scan = None

keyframe_lidar_x = 0.0
keyframe_lidar_y = 0.0
keyframe_heading = 0.0

localization_valid = False
localization_state = "STARTING"

last_icp_error = 999.0
last_inlier_ratio = 0.0
last_translation_m = 0.0

# Every LiDAR localization update gets a new sample id, even
# when X/Y stays unchanged. Java fusion uses this to know that
# a fresh LiDAR observation arrived after wheel slip/collision.
lidar_sample_id = 0

# Incremented whenever LiDAR ICP is manually reset. Java fusion
# uses this to safely capture a new LiDAR reference frame.
lidar_reset_id = 0


# ============================================================
# LIVE LIDAR STATE
# ============================================================

latest_angles = np.array(
    [],
    dtype=float
)

latest_distances_mm = np.array(
    [],
    dtype=float
)

latest_local_points = np.empty(
    (0, 2),
    dtype=float
)

nearest_obstacle_distance = 9999.0
nearest_obstacle_angle = 0.0


# ============================================================
# MAP / ROUTE / LOG
# ============================================================

occupancy_evidence = np.zeros(
    (
        GRID_CELLS,
        GRID_CELLS
    ),
    dtype=np.int16
)

route_x = [
    0.0
]

route_y = [
    0.0
]

log_rows = []
sample_counter = 0
test_start_time = time.monotonic()

map_frame_counter = 0


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
# KEYBOARD DRIVE
# ============================================================

def calculate_drive():

    if space_stop_active:
        return (
            0.0,
            0.0,
            0.0
        )

    x = 0.0
    y = 0.0
    z = 0.0

    if "w" in pressed_keys:
        y += MOVE_SPEED

    if "s" in pressed_keys:
        y -= MOVE_SPEED

    if "a" in pressed_keys:
        x -= MOVE_SPEED

    if "d" in pressed_keys:
        x += MOVE_SPEED

    if "q" in pressed_keys:
        z -= TURN_SPEED

    if "e" in pressed_keys:
        z += TURN_SPEED

    return (
        x,
        y,
        z
    )


def translation_commanded():

    return (
        abs(current_command_x)
        >=
        COMMAND_MOVE_THRESHOLD
        or
        abs(current_command_y)
        >=
        COMMAND_MOVE_THRESHOLD
    )


def rotation_commanded():

    return (
        abs(current_command_z)
        >=
        COMMAND_TURN_THRESHOLD
    )


def pure_rotation_commanded():

    return (
        rotation_commanded()
        and
        not translation_commanded()
    )


def keyboard_window_has_focus():

    try:
        return (
            root.focus_displayof()
            is not None
        )

    except tk.TclError:
        return False


def send_stop():

    global current_command_x
    global current_command_y
    global current_command_z

    current_command_x = 0.0
    current_command_y = 0.0
    current_command_z = 0.0

    keyboard_table.putNumber(
        "X",
        0.0
    )

    keyboard_table.putNumber(
        "Y",
        0.0
    )

    keyboard_table.putNumber(
        "Z",
        0.0
    )


def update_drive():

    global heartbeat

    global current_command_x
    global current_command_y
    global current_command_z

    if not program_running:
        return

    heartbeat += 1

    keyboard_table.putNumber(
        "Heartbeat",
        heartbeat
    )

    if not keyboard_window_has_focus():

        pressed_keys.clear()

        keyboard_table.putBoolean(
            "Enabled",
            False
        )

        send_stop()

        drive_status_label.config(
            text="STOPPED - WINDOW NOT FOCUSED"
        )

        root.after(
            DRIVE_UPDATE_MS,
            update_drive
        )

        return

    keyboard_table.putBoolean(
        "Enabled",
        True
    )

    (
        current_command_x,
        current_command_y,
        current_command_z
    ) = calculate_drive()

    keyboard_table.putNumber(
        "X",
        current_command_x
    )

    keyboard_table.putNumber(
        "Y",
        current_command_y
    )

    keyboard_table.putNumber(
        "Z",
        current_command_z
    )

    x_command_label.config(
        text=(
            f"X Cmd: "
            f"{current_command_x:.2f}"
        )
    )

    y_command_label.config(
        text=(
            f"Y Cmd: "
            f"{current_command_y:.2f}"
        )
    )

    z_command_label.config(
        text=(
            f"Z Cmd: "
            f"{current_command_z:.2f}"
        )
    )

    if space_stop_active:

        drive_status_label.config(
            text="STOPPED - SPACE"
        )

    elif (
        translation_commanded()
        and
        rotation_commanded()
    ):

        drive_status_label.config(
            text="MOVING + TURNING"
        )

    elif translation_commanded():

        drive_status_label.config(
            text="MOVING"
        )

    elif rotation_commanded():

        drive_status_label.config(
            text="ROTATING"
        )

    else:

        drive_status_label.config(
            text="READY"
        )

    root.after(
        DRIVE_UPDATE_MS,
        update_drive
    )


# ============================================================
# NAVX
# ============================================================

def read_raw_navx_heading():

    value = robot_pose_table.getNumber(
        "Heading",
        9999.0
    )

    if value == 9999.0:
        return None

    return float(
        value
    )


def get_relative_navx_heading():

    global navx_zero_offset
    global navx_zero_initialized

    raw_heading = read_raw_navx_heading()

    if raw_heading is None:
        return None

    if not navx_zero_initialized:

        navx_zero_offset = (
            raw_heading
        )

        navx_zero_initialized = True

    relative = (
        raw_heading
        -
        navx_zero_offset
    )

    relative *= (
        NAVX_HEADING_SIGN
    )

    return normalize_heading(
        relative
    )


def reset_lidar_navx_zero():

    global navx_zero_offset
    global navx_zero_initialized

    raw_heading = read_raw_navx_heading()

    if raw_heading is None:

        navx_zero_initialized = False
        navx_zero_offset = 0.0

        return

    navx_zero_offset = (
        raw_heading
    )

    navx_zero_initialized = True


# ============================================================
# LIDAR SCAN
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
            MIN_DISTANCE_MM
            <=
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
            np.array(
                [],
                dtype=float
            ),
            np.array(
                [],
                dtype=float
            )
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


def scan_to_xy(
        angles,
        distances_mm):

    if len(angles) == 0:

        return np.empty(
            (0, 2)
        )

    angle_rad = np.radians(
        angles
    )

    distance_m = (
        distances_mm
        /
        1000.0
    )

    # Robot local frame:
    #
    # +X = right
    # +Y = forward
    #
    # LiDAR:
    #
    # 0° = forward
    # 90° = right

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
# ICP
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

    known_rotation = heading_rotation_matrix(
        known_heading_delta_deg
    )

    transformed = (
        current
        @
        known_rotation.T
    )

    total_translation = np.zeros(
        2,
        dtype=float
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

        if valid_count < minimum_matches:

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

            if (
                abs(
                    previous_error
                    -
                    error
                )
                <
                0.0001
            ):

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


def local_translation_to_world(
        local_translation,
        heading_deg):

    local_x = float(
        local_translation[0]
    )

    local_y = float(
        local_translation[1]
    )

    heading_rad = math.radians(
        heading_deg
    )

    world_x = (
        local_x
        *
        math.cos(
            heading_rad
        )
        +
        local_y
        *
        math.sin(
            heading_rad
        )
    )

    world_y = (
        -local_x
        *
        math.sin(
            heading_rad
        )
        +
        local_y
        *
        math.cos(
            heading_rad
        )
    )

    return np.array(
        [
            world_x,
            world_y
        ],
        dtype=float
    )


# ============================================================
# LIDAR LOCALIZATION
# ============================================================

def reset_lidar_localization():

    global lidar_robot_x
    global lidar_robot_y
    global lidar_robot_heading

    global previous_navx_heading
    global previous_good_scan

    global keyframe_scan
    global keyframe_lidar_x
    global keyframe_lidar_y
    global keyframe_heading

    global localization_valid
    global localization_state

    global last_icp_error
    global last_inlier_ratio
    global last_translation_m

    global lidar_reset_id

    # Tell Java fusion that the LiDAR coordinate reference has
    # intentionally restarted. It will capture a new alignment
    # instead of treating this as a giant localization jump.
    lidar_reset_id += 1

    lidar_robot_x = 0.0
    lidar_robot_y = 0.0
    lidar_robot_heading = 0.0

    reset_lidar_navx_zero()

    previous_navx_heading = 0.0
    previous_good_scan = None

    keyframe_scan = None
    keyframe_lidar_x = 0.0
    keyframe_lidar_y = 0.0
    keyframe_heading = 0.0

    localization_valid = False
    localization_state = "RESET"

    last_icp_error = 999.0
    last_inlier_ratio = 0.0
    last_translation_m = 0.0

    # Publish the reset state immediately instead of waiting for
    # the next tracking loop. This prevents Java fusion from
    # seeing the old LiDAR frame after C is pressed.
    publish_lidar_localization()


def create_keyframe(
        scan,
        heading):

    global keyframe_scan
    global keyframe_lidar_x
    global keyframe_lidar_y
    global keyframe_heading

    keyframe_scan = (
        scan.copy()
    )

    keyframe_lidar_x = (
        lidar_robot_x
    )

    keyframe_lidar_y = (
        lidar_robot_y
    )

    keyframe_heading = (
        heading
    )


def should_create_keyframe(
        heading):

    if keyframe_scan is None:
        return True

    translation_since_keyframe = math.hypot(
        lidar_robot_x
        -
        keyframe_lidar_x,

        lidar_robot_y
        -
        keyframe_lidar_y
    )

    rotation_since_keyframe = abs(
        heading_difference(
            heading,
            keyframe_heading
        )
    )

    return (
        translation_since_keyframe
        >=
        KEYFRAME_TRANSLATION_M
        or
        rotation_since_keyframe
        >=
        KEYFRAME_ROTATION_DEG
    )


def publish_lidar_localization():

    lidar_localization_table.putNumber(
        "RobotX",
        lidar_robot_x
    )

    lidar_localization_table.putNumber(
        "RobotY",
        lidar_robot_y
    )

    lidar_localization_table.putNumber(
        "RobotHeading",
        lidar_robot_heading
    )

    lidar_localization_table.putBoolean(
        "LocalizationValid",
        localization_valid
    )

    lidar_localization_table.putString(
        "State",
        localization_state
    )

    lidar_localization_table.putNumber(
        "ICPError",
        last_icp_error
    )

    lidar_localization_table.putNumber(
        "InlierRatio",
        last_inlier_ratio
    )

    lidar_localization_table.putNumber(
        "SampleId",
        lidar_sample_id
    )

    lidar_localization_table.putNumber(
        "ResetId",
        lidar_reset_id
    )


def update_lidar_localization(
        current_scan):

    global lidar_robot_x
    global lidar_robot_y
    global lidar_robot_heading

    global previous_navx_heading
    global previous_good_scan

    global localization_valid
    global localization_state

    global last_icp_error
    global last_inlier_ratio
    global last_translation_m

    global lidar_sample_id

    # IMPORTANT: a fresh scan must be identifiable even if ICP
    # concludes that X/Y did not move. This is what allows Java
    # fusion to correct encoder wheel-slip after hitting a box.
    lidar_sample_id += 1

    relative_heading = (
        get_relative_navx_heading()
    )

    if relative_heading is None:

        localization_valid = False
        localization_state = "NO_NAVX"

        publish_lidar_localization()

        return

    lidar_robot_heading = (
        relative_heading
    )

    if len(current_scan) < MIN_SCAN_POINTS:

        localization_valid = False
        localization_state = "NOT_ENOUGH_POINTS"

        publish_lidar_localization()

        return

    if previous_good_scan is None:

        previous_good_scan = (
            current_scan.copy()
        )

        previous_navx_heading = (
            relative_heading
        )

        create_keyframe(
            current_scan,
            relative_heading
        )

        localization_valid = True
        localization_state = "INITIALIZED"

        publish_lidar_localization()

        return

    heading_delta = heading_difference(
        relative_heading,
        previous_navx_heading
    )

    # --------------------------------------------------------
    # IDLE:
    #
    # Refresh LiDAR reference but do not invent translation.
    #
    # This prevents stationary ICP drift from feeding fusion.
    # --------------------------------------------------------

    if (
        not translation_commanded()
        and
        not rotation_commanded()
    ):

        previous_good_scan = (
            current_scan.copy()
        )

        previous_navx_heading = (
            relative_heading
        )

        localization_valid = True
        localization_state = "IDLE_HOLD"

        last_translation_m = 0.0

        publish_lidar_localization()

        return

    # --------------------------------------------------------
    # PURE Q / E:
    #
    # navX handles heading.
    # Refresh LiDAR reference and keep LiDAR X/Y fixed.
    #
    # The FUSION pose is still allowed to move slightly from
    # real encoder translation during rotation.
    # --------------------------------------------------------

    if pure_rotation_commanded():

        previous_good_scan = (
            current_scan.copy()
        )

        previous_navx_heading = (
            relative_heading
        )

        localization_valid = True
        localization_state = "PURE_ROTATION"

        last_translation_m = 0.0

        if should_create_keyframe(
            relative_heading
        ):

            create_keyframe(
                current_scan,
                relative_heading
            )

        publish_lidar_localization()

        return

    # --------------------------------------------------------
    # NORMAL ICP
    #
    # Includes:
    # W/S/A/D
    # W+Q / W+E
    # A+Q / A+E
    # etc.
    # --------------------------------------------------------

    (
        relative_translation,
        matched,
        error,
        inlier_ratio
    ) = translation_icp(
        current_scan,
        previous_good_scan,
        heading_delta,
        NORMAL_ICP_POINTS,
        NORMAL_ICP_ITERATIONS,
        NORMAL_CORRESPONDENCE_M,
        NORMAL_MIN_MATCHES
    )

    accepted = (
        matched
        and
        translation_quality_good(
            relative_translation,
            error,
            inlier_ratio,
            recovery=False
        )
    )

    used_recovery = False

    # --------------------------------------------------------
    # KEYFRAME RECOVERY
    # --------------------------------------------------------

    if (
        not accepted
        and
        keyframe_scan is not None
    ):

        keyframe_heading_delta = (
            heading_difference(
                relative_heading,
                keyframe_heading
            )
        )

        (
            keyframe_translation,
            recovery_matched,
            recovery_error,
            recovery_ratio
        ) = translation_icp(
            current_scan,
            keyframe_scan,
            keyframe_heading_delta,
            RECOVERY_ICP_POINTS,
            RECOVERY_ICP_ITERATIONS,
            RECOVERY_CORRESPONDENCE_M,
            RECOVERY_MIN_MATCHES
        )

        if (
            recovery_matched
            and
            translation_quality_good(
                keyframe_translation,
                recovery_error,
                recovery_ratio,
                recovery=True
            )
        ):

            # ICP transform aligns CURRENT scan back to the
            # keyframe. Robot movement is opposite transform.
            local_robot_movement = (
                -keyframe_translation
            )

            world_movement = (
                local_translation_to_world(
                    local_robot_movement,
                    keyframe_heading
                )
            )

            lidar_robot_x = (
                keyframe_lidar_x
                +
                world_movement[0]
            )

            lidar_robot_y = (
                keyframe_lidar_y
                +
                world_movement[1]
            )

            error = recovery_error
            inlier_ratio = recovery_ratio

            accepted = True
            used_recovery = True

    if accepted:

        if not used_recovery:

            # ICP translation tells how current scan must be
            # shifted to match the previous scan.
            # Robot movement is opposite that shift.
            local_robot_movement = (
                -relative_translation
            )

            movement_m = float(
                np.linalg.norm(
                    local_robot_movement
                )
            )

            if (
                movement_m
                <
                STATIONARY_TRANSLATION_M
            ):

                local_robot_movement = (
                    np.zeros(
                        2,
                        dtype=float
                    )
                )

            world_movement = (
                local_translation_to_world(
                    local_robot_movement,
                    previous_navx_heading
                )
            )

            lidar_robot_x += (
                world_movement[0]
            )

            lidar_robot_y += (
                world_movement[1]
            )

        last_translation_m = float(
            np.linalg.norm(
                relative_translation
            )
        )

        last_icp_error = (
            error
        )

        last_inlier_ratio = (
            inlier_ratio
        )

        localization_valid = True

        if used_recovery:
            localization_state = "RECOVERY"
        elif rotation_commanded():
            localization_state = "TRACKING_TURN"
        else:
            localization_state = "TRACKING"

        previous_good_scan = (
            current_scan.copy()
        )

        previous_navx_heading = (
            relative_heading
        )

        if should_create_keyframe(
            relative_heading
        ):

            create_keyframe(
                current_scan,
                relative_heading
            )

    else:

        localization_valid = False
        localization_state = "HOLD_BAD_ICP"

        # Refreshing the reference after a bad match avoids
        # repeatedly trying to align against a very old scan.
        previous_good_scan = (
            current_scan.copy()
        )

        previous_navx_heading = (
            relative_heading
        )

    publish_lidar_localization()


# ============================================================
# FUSION POSE
# ============================================================

def read_fusion_pose():

    global raw_fused_x
    global raw_fused_y
    global raw_fused_heading
    global fusion_active

    fusion_active = fusion_table.getBoolean(
        "Active",
        False
    )

    if not fusion_active:
        return False

    x = fusion_table.getNumber(
        "X",
        9999.0
    )

    y = fusion_table.getNumber(
        "Y",
        9999.0
    )

    heading = fusion_table.getNumber(
        "Heading",
        9999.0
    )

    if (
        x == 9999.0
        or
        y == 9999.0
        or
        heading == 9999.0
    ):

        return False

    raw_fused_x = float(
        x
    )

    raw_fused_y = float(
        y
    )

    raw_fused_heading = float(
        heading
    )

    return True


def initialize_fusion_zero():

    # Compatibility helper.
    #
    # In the corrected design the Java FusionLocalization
    # subsystem owns the actual zero frame. When fusion starts
    # or C is pressed, Java resets:
    #
    # fused X       = 0
    # fused Y       = 0
    # fused Heading = 0
    #
    # Therefore Python does NOT create another rotated zero frame.

    global fusion_zero_initialized
    global fusion_zero_x
    global fusion_zero_y
    global fusion_zero_heading

    if not read_fusion_pose():
        return False

    fusion_zero_x = 0.0
    fusion_zero_y = 0.0
    fusion_zero_heading = 0.0

    fusion_zero_initialized = True

    return True


def raw_fusion_world_to_zero_frame(
        world_x,
        world_y):

    # Java fusion coordinates are already relative to the latest
    # synchronized fusion reset. Do NOT rotate them again.
    #
    # Internal fusion frame:
    #   +X = robot-right at reset
    #   +Y = robot-forward at reset

    return (
        float(world_x),
        float(world_y)
    )


def zero_frame_to_display(
        local_x,
        local_y):

    # Display convention:
    #
    # fused +Y (forward at reset) -> display +X / right
    # fused +X (right at reset)   -> display -Y / down

    display_x = (
        local_y
    )

    display_y = (
        -local_x
    )

    return (
        display_x,
        display_y
    )


def update_display_robot_pose():

    global display_robot_x
    global display_robot_y
    global display_robot_heading

    if not read_fusion_pose():
        return False

    (
        display_robot_x,
        display_robot_y
    ) = zero_frame_to_display(
        raw_fused_x,
        raw_fused_y
    )

    # Fused heading is already navX heading relative to the
    # synchronized fusion reset.
    display_robot_heading = normalize_heading(
        raw_fused_heading
    )

    return True


# ============================================================
# FUSED POSE + LIDAR -> WORLD / DISPLAY OBSTACLES
# ============================================================

def local_lidar_points_to_raw_fusion_world(
        local_points):

    if len(local_points) == 0:

        return np.empty(
            (0, 2)
        )

    heading_rad = math.radians(
        raw_fused_heading
    )

    cos_h = math.cos(
        heading_rad
    )

    sin_h = math.sin(
        heading_rad
    )

    local_x = local_points[
        :,
        0
    ]

    local_y = local_points[
        :,
        1
    ]

    world_x = (
        raw_fused_x
        +
        local_x * cos_h
        +
        local_y * sin_h
    )

    world_y = (
        raw_fused_y
        -
        local_x * sin_h
        +
        local_y * cos_h
    )

    return np.column_stack(
        (
            world_x,
            world_y
        )
    )


def raw_world_points_to_display(
        raw_world_points):

    if len(raw_world_points) == 0:

        return np.empty(
            (0, 2)
        )

    # Raw fused world uses:
    #   +X = reset-right
    #   +Y = reset-forward
    #
    # Display uses:
    #   +X = reset-forward
    #   +Y = reset-left

    display_x = (
        raw_world_points[
            :,
            1
        ]
    )

    display_y = (
        -raw_world_points[
            :,
            0
        ]
    )

    return np.column_stack(
        (
            display_x,
            display_y
        )
    )


def display_heading_vector(
        heading_deg,
        length):

    # One shared heading convention for BOTH:
    #
    # - robot arrow
    # - LiDAR obstacle placement on the map
    #
    # 0°   = right / +Display X
    # 90°  = down  / -Display Y
    # 180° = left
    # -90° = up

    heading_rad = math.radians(
        heading_deg
    )

    return (
        length
        *
        math.cos(
            heading_rad
        ),
        -length
        *
        math.sin(
            heading_rad
        )
    )


def local_lidar_points_to_display_from_fused_pose(
        local_points):

    # Transform LiDAR-local points using the SAME fused pose and
    # fused heading used by the route marker and map arrow.
    #
    # LiDAR local:
    #   +X = robot right
    #   +Y = robot forward
    #
    # Fused internal:
    #   +X = reset-right
    #   +Y = reset-forward
    #
    # Display:
    #   +X = reset-forward
    #   +Y = reset-left

    if len(local_points) == 0:

        return np.empty(
            (0, 2)
        )

    heading_rad = math.radians(
        raw_fused_heading
    )

    cos_h = math.cos(
        heading_rad
    )

    sin_h = math.sin(
        heading_rad
    )

    local_x = local_points[
        :,
        0
    ]

    local_y = local_points[
        :,
        1
    ]

    # Local robot -> fused internal frame.
    fused_world_x = (
        raw_fused_x
        +
        local_x
        *
        cos_h
        +
        local_y
        *
        sin_h
    )

    fused_world_y = (
        raw_fused_y
        -
        local_x
        *
        sin_h
        +
        local_y
        *
        cos_h
    )

    # Fused internal -> display.
    display_x = (
        fused_world_y
    )

    display_y = (
        -fused_world_x
    )

    return np.column_stack(
        (
            display_x,
            display_y
        )
    )


# ============================================================
# NEAREST OBSTACLE
# ============================================================

def update_nearest_obstacle(
        angles,
        distances_mm):

    global nearest_obstacle_distance
    global nearest_obstacle_angle

    if len(distances_mm) == 0:

        nearest_obstacle_distance = (
            9999.0
        )

        nearest_obstacle_angle = (
            0.0
        )

        return

    index = int(
        np.argmin(
            distances_mm
        )
    )

    nearest_obstacle_distance = float(
        distances_mm[
            index
        ]
        /
        1000.0
    )

    nearest_obstacle_angle = float(
        angles[
            index
        ]
    )

    lidar_localization_table.putNumber(
        "ObstacleDistance",
        nearest_obstacle_distance
    )

    lidar_localization_table.putNumber(
        "ObstacleAngle",
        nearest_obstacle_angle
    )


# ============================================================
# OCCUPANCY GRID HELPERS
# ============================================================

def display_to_grid(
        display_x,
        display_y):

    half = (
        MAP_SIZE_M
        /
        2.0
    )

    gx = int(
        math.floor(
            (
                display_x
                +
                half
            )
            /
            GRID_RESOLUTION_M
        )
    )

    gy = int(
        math.floor(
            (
                display_y
                +
                half
            )
            /
            GRID_RESOLUTION_M
        )
    )

    return (
        gx,
        gy
    )


def grid_inside(
        gx,
        gy):

    return (
        0
        <=
        gx
        <
        GRID_CELLS
        and
        0
        <=
        gy
        <
        GRID_CELLS
    )


def bresenham(
        x0,
        y0,
        x1,
        y1):

    points = []

    dx = abs(
        x1
        -
        x0
    )

    dy = abs(
        y1
        -
        y0
    )

    x = x0
    y = y0

    sx = (
        1
        if x0 < x1
        else
        -1
    )

    sy = (
        1
        if y0 < y1
        else
        -1
    )

    if dx > dy:

        error = (
            dx
            /
            2
        )

        while x != x1:

            points.append(
                (
                    x,
                    y
                )
            )

            error -= dy

            if error < 0:

                y += sy
                error += dx

            x += sx

    else:

        error = (
            dy
            /
            2
        )

        while y != y1:

            points.append(
                (
                    x,
                    y
                )
            )

            error -= dx

            if error < 0:

                x += sx
                error += dy

            y += sy

    points.append(
        (
            x1,
            y1
        )
    )

    return points


def update_occupancy_map(
        display_points):

    global occupancy_evidence

    if len(display_points) == 0:
        return

    (
        robot_gx,
        robot_gy
    ) = display_to_grid(
        display_robot_x,
        display_robot_y
    )

    if not grid_inside(
        robot_gx,
        robot_gy
    ):

        return

    # Downsample map rays to reduce CPU load.
    if len(display_points) > 220:

        indices = np.linspace(
            0,
            len(display_points) - 1,
            220,
            dtype=int
        )

        map_points = display_points[
            indices
        ]

    else:

        map_points = (
            display_points
        )

    for point in map_points:

        (
            hit_gx,
            hit_gy
        ) = display_to_grid(
            float(
                point[0]
            ),
            float(
                point[1]
            )
        )

        if not grid_inside(
            hit_gx,
            hit_gy
        ):

            continue

        ray = bresenham(
            robot_gx,
            robot_gy,
            hit_gx,
            hit_gy
        )

        # Free cells before obstacle.
        for (
            gx,
            gy
        ) in ray[:-1]:

            if grid_inside(
                gx,
                gy
            ):

                occupancy_evidence[
                    gy,
                    gx
                ] = max(
                    MIN_GRID_EVIDENCE,
                    occupancy_evidence[
                        gy,
                        gx
                    ]
                    +
                    FREE_UPDATE
                )

        # Obstacle endpoint.
        occupancy_evidence[
            hit_gy,
            hit_gx
        ] = min(
            MAX_GRID_EVIDENCE,
            occupancy_evidence[
                hit_gy,
                hit_gx
            ]
            +
            OCCUPIED_UPDATE
        )


def get_occupancy_image():

    image = np.full(
        (
            GRID_CELLS,
            GRID_CELLS
        ),
        0.5,
        dtype=float
    )

    free_cells = (
        occupancy_evidence
        <=
        FREE_THRESHOLD
    )

    occupied_cells = (
        occupancy_evidence
        >=
        OCCUPIED_THRESHOLD
    )

    image[
        free_cells
    ] = 1.0

    image[
        occupied_cells
    ] = 0.0

    return image


# ============================================================
# LOGGING
# ============================================================

def append_route_and_log():

    global sample_counter

    route_x.append(
        display_robot_x
    )

    route_y.append(
        display_robot_y
    )

    if len(route_x) > ROUTE_MAX_POINTS:

        del route_x[
            :len(route_x) - ROUTE_MAX_POINTS
        ]

        del route_y[
            :len(route_y) - ROUTE_MAX_POINTS
        ]

    sample_counter += 1

    elapsed = (
        time.monotonic()
        -
        test_start_time
    )

    row = {
        "sample": sample_counter,
        "elapsed": elapsed,
        "x": display_robot_x,
        "y": display_robot_y,
        "heading": display_robot_heading,
        "cmd_x": current_command_x,
        "cmd_y": current_command_y,
        "cmd_z": current_command_z,
        "lidar_valid": localization_valid
    }

    log_rows.append(
        row
    )

    log_table.insert(
        "",
        "end",
        values=(
            row[
                "sample"
            ],
            f'{row["elapsed"]:.1f}',
            f'{row["x"]:.3f}',
            f'{row["y"]:.3f}',
            f'{row["heading"]:.1f}',
            (
                "YES"
                if row[
                    "lidar_valid"
                ]
                else
                "NO"
            )
        )
    )

    children = (
        log_table.get_children()
    )

    while (
        len(children)
        >
        MAX_VISIBLE_LOG_ROWS
    ):

        log_table.delete(
            children[
                0
            ]
        )

        children = (
            log_table.get_children()
        )

    children = (
        log_table.get_children()
    )

    if len(children) > 0:

        log_table.see(
            children[
                -1
            ]
        )


def save_csv():

    if len(log_rows) == 0:

        save_status_label.config(
            text="Nothing to save yet."
        )

        return

    timestamp = time.strftime(
        "%Y%m%d_%H%M%S"
    )

    filename = (
        "fusion_localization_log_"
        +
        timestamp
        +
        ".csv"
    )

    path = os.path.join(
        SCRIPT_FOLDER,
        filename
    )

    with open(
        path,
        "w",
        newline=""
    ) as file:

        writer = csv.writer(
            file
        )

        writer.writerow(
            [
                "Sample",
                "Elapsed_s",
                "X_m",
                "Y_m",
                "Heading_deg",
                "Command_X",
                "Command_Y",
                "Command_Z",
                "LidarLocalizationValid"
            ]
        )

        for row in log_rows:

            writer.writerow(
                [
                    row[
                        "sample"
                    ],
                    row[
                        "elapsed"
                    ],
                    row[
                        "x"
                    ],
                    row[
                        "y"
                    ],
                    row[
                        "heading"
                    ],
                    row[
                        "cmd_x"
                    ],
                    row[
                        "cmd_y"
                    ],
                    row[
                        "cmd_z"
                    ],
                    row[
                        "lidar_valid"
                    ]
                ]
            )

    save_status_label.config(
        text=(
            "Saved:\n"
            +
            filename
        )
    )


def save_map():

    timestamp = time.strftime(
        "%Y%m%d_%H%M%S"
    )

    npy_filename = (
        "fusion_occupancy_"
        +
        timestamp
        +
        ".npy"
    )

    csv_filename = (
        "fusion_occupied_cells_"
        +
        timestamp
        +
        ".csv"
    )

    npy_path = os.path.join(
        MAP_SAVE_FOLDER,
        npy_filename
    )

    csv_path = os.path.join(
        MAP_SAVE_FOLDER,
        csv_filename
    )

    np.save(
        npy_path,
        occupancy_evidence
    )

    half = (
        MAP_SIZE_M
        /
        2.0
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
                "DisplayX_m",
                "DisplayY_m",
                "Evidence"
            ]
        )

        occupied_indices = np.argwhere(
            occupancy_evidence
            >=
            OCCUPIED_THRESHOLD
        )

        for (
            gy,
            gx
        ) in occupied_indices:

            x = (
                (
                    gx
                    +
                    0.5
                )
                *
                GRID_RESOLUTION_M
                -
                half
            )

            y = (
                (
                    gy
                    +
                    0.5
                )
                *
                GRID_RESOLUTION_M
                -
                half
            )

            writer.writerow(
                [
                    x,
                    y,
                    int(
                        occupancy_evidence[
                            gy,
                            gx
                        ]
                    )
                ]
            )

    save_status_label.config(
        text=(
            "Map saved:\n"
            +
            npy_filename
        )
    )


# ============================================================
# RESET DISPLAY / MAP
# ============================================================

def reset_display_and_map():

    global fusion_reset_request_id
    global fusion_reset_pending
    global fusion_reset_requested_at

    global fusion_zero_initialized
    global fusion_zero_x
    global fusion_zero_y
    global fusion_zero_heading

    global display_robot_x
    global display_robot_y
    global display_robot_heading

    global route_x
    global route_y

    global log_rows
    global sample_counter
    global test_start_time

    global occupancy_evidence

    # --------------------------------------------------------
    # STOP FIRST
    # --------------------------------------------------------

    pressed_keys.clear()

    send_stop()

    keyboard_table.putBoolean(
        "Enabled",
        True
    )

    # --------------------------------------------------------
    # RESET LIDAR ICP FRAME NOW
    #
    # reset_lidar_localization() publishes its ResetId
    # immediately in this version.
    # --------------------------------------------------------

    reset_lidar_localization()

    # --------------------------------------------------------
    # CREATE A UNIQUE RESET TOKEN
    #
    # Use current time in milliseconds instead of simply +1.
    # This remains unique even if Python is restarted and old
    # NetworkTables values are still present.
    # --------------------------------------------------------

    current_request = int(
        fusion_table.getNumber(
            "ResetRequestId",
            0.0
        )
    )

    current_ack = int(
        fusion_table.getNumber(
            "ResetAckId",
            0.0
        )
    )

    time_token = int(
        time.time()
        *
        1000.0
    )

    fusion_reset_request_id = max(
        current_request + 1,
        current_ack + 1,
        time_token
    )

    # --------------------------------------------------------
    # ASK JAVA TO RESET THE REAL FUSION FRAME
    #
    # Two fields are used deliberately:
    #
    # ResetRequested = explicit command
    # ResetRequestId = unique request token
    #
    # Java acknowledges with ResetAckId.
    # --------------------------------------------------------

    fusion_table.putNumber(
        "ResetRequestId",
        fusion_reset_request_id
    )

    fusion_table.putBoolean(
        "ResetRequested",
        True
    )

    fusion_reset_pending = True
    fusion_reset_requested_at = (
        time.monotonic()
    )

    # --------------------------------------------------------
    # HOLD THE GUI AT ZERO WHILE WAITING FOR JAVA ACK
    #
    # This fixes the previous behaviour where the graph was
    # cleared and then immediately jumped back to the old Java
    # fusion pose before the reset request had been processed.
    # --------------------------------------------------------

    fusion_zero_initialized = True
    fusion_zero_x = 0.0
    fusion_zero_y = 0.0
    fusion_zero_heading = 0.0

    display_robot_x = 0.0
    display_robot_y = 0.0
    display_robot_heading = 0.0

    route_x = [
        0.0
    ]

    route_y = [
        0.0
    ]

    log_rows = []
    sample_counter = 0
    test_start_time = (
        time.monotonic()
    )

    occupancy_evidence = np.zeros(
        (
            GRID_CELLS,
            GRID_CELLS
        ),
        dtype=np.int16
    )

    for item in log_table.get_children():

        log_table.delete(
            item
        )

    # Force visible artists to zero immediately.
    route_line.set_data(
        route_x,
        route_y
    )

    map_robot.set_data(
        [
            0.0
        ],
        [
            0.0
        ]
    )

    heading_arrow.set_positions(
        (
            0.0,
            0.0
        ),
        (
            HEADING_ARROW_LENGTH_M,
            0.0
        )
    )

    map_image.set_data(
        get_occupancy_image()
    )

    save_status_label.config(
        text=(
            "RESETTING...\n"
            "Waiting for Java fusion reset acknowledgement."
        )
    )

    fused_status_label.config(
        text="FUSION: RESETTING"
    )

    canvas.draw_idle()


# ============================================================
# MAIN TRACKING LOOP
# ============================================================

def tracking_update():

    global latest_angles
    global latest_distances_mm
    global latest_local_points

    global map_frame_counter

    global display_robot_x
    global display_robot_y
    global display_robot_heading

    if not program_running:
        return

    # --------------------------------------------------------
    # FULL RESET HANDSHAKE
    #
    # Do not write the persistent map while waiting for Java
    # fusion to acknowledge the synchronized reset.
    # --------------------------------------------------------

    global fusion_reset_pending

    if fusion_reset_pending:

        reset_ack_id = int(
            fusion_table.getNumber(
                "ResetAckId",
                -1.0
            )
        )

        if reset_ack_id == fusion_reset_request_id:

            fusion_reset_pending = False

            # Java has now captured the CURRENT encoder pose
            # and CURRENT navX heading as its real zero.
            #
            # Read the fresh fusion pose only after ACK.
            read_fusion_pose()

            display_robot_x = 0.0
            display_robot_y = 0.0
            display_robot_heading = 0.0

            save_status_label.config(
                text=(
                    "RESET COMPLETE.\n"
                    "Fusion X/Y/Heading = 0,0,0."
                )
            )

            fused_status_label.config(
                text="FUSION: ACTIVE"
            )

        elif (
            time.monotonic()
            -
            fusion_reset_requested_at
            >
            FUSION_RESET_TIMEOUT_S
        ):

            # Do NOT fake a successful reset.
            #
            # Keep the map held and tell the user the Java side
            # did not acknowledge the request.
            save_status_label.config(
                text=(
                    "RESET FAILED - NO JAVA ACK.\n"
                    "Check FUSION_LOCALIZATION mode / deployment."
                )
            )

            fused_status_label.config(
                text="FUSION: RESET FAILED"
            )

        else:

            # While pending:
            # - keep fused route/map at 0
            # - continue showing live robot-relative LiDAR
            # - do not read the OLD fused pose back into the GUI
            display_robot_x = 0.0
            display_robot_y = 0.0
            display_robot_heading = 0.0

    (
        angles,
        distances_mm
    ) = get_full_scan()

    latest_angles = (
        angles
    )

    latest_distances_mm = (
        distances_mm
    )

    local_points = scan_to_xy(
        angles,
        distances_mm
    )

    latest_local_points = (
        local_points
    )

    update_nearest_obstacle(
        angles,
        distances_mm
    )

    # --------------------------------------------------------
    # 1. LiDAR-only localization estimate.
    #
    # Publishes to LidarLocalization.
    # Java fusion reads that as a correction source.
    # --------------------------------------------------------

    update_lidar_localization(
        local_points
    )

    # --------------------------------------------------------
    # 2. Read final fused pose from Java.
    # --------------------------------------------------------

    if fusion_reset_pending:

        fusion_pose_ok = False

    else:

        fusion_pose_ok = (
            update_display_robot_pose()
        )

    # --------------------------------------------------------
    # 3. Transform LIVE LiDAR obstacles using FUSED pose.
    # --------------------------------------------------------

    if fusion_pose_ok:

        # ----------------------------------------------------
        # IMPORTANT:
        #
        # Map obstacles are transformed using EXACTLY the same
        # fused X/Y/heading that drives the orange robot marker
        # and heading arrow. This keeps the arrow, route and
        # obstacle map in one coordinate frame.
        # ----------------------------------------------------

        display_world_points = (
            local_lidar_points_to_display_from_fused_pose(
                local_points
            )
        )

        correction_error_for_map = fusion_table.getNumber(
            "CorrectionErrorM",
            0.0
        )

        correction_applied_for_map = fusion_table.getNumber(
            "CorrectionAppliedM",
            0.0
        )

        map_pose_stable = (
            not fusion_reset_pending
            and
            correction_error_for_map
            <=
            MAP_MAX_FUSION_ERROR_M
            and
            correction_applied_for_map
            <=
            MAP_MAX_CORRECTION_APPLIED_M
        )

        map_frame_counter += 1

        if (
            map_pose_stable
            and
            map_frame_counter
            %
            MAP_UPDATE_EVERY_N_FRAMES
            ==
            0
        ):

            update_occupancy_map(
                display_world_points
            )

        # Route records the actual fused pose even while map
        # writing is paused during a large correction.
        append_route_and_log()

    else:

        display_world_points = np.empty(
            (0, 2)
        )

        map_pose_stable = False

    # --------------------------------------------------------
    # LIVE LIDAR PANEL
    #
    # Use local display coordinates:
    #
    # LiDAR/robot forward = screen +X
    # robot right         = screen -Y
    # --------------------------------------------------------

    if len(local_points) > 0:

        live_display_x = (
            local_points[
                :,
                1
            ]
        )

        live_display_y = (
            -local_points[
                :,
                0
            ]
        )

        live_points = np.column_stack(
            (
                live_display_x,
                live_display_y
            )
        )

        live_scatter.set_offsets(
            live_points
        )

    else:

        live_scatter.set_offsets(
            np.empty(
                (
                    0,
                    2
                )
            )
        )

    # Robot stays at centre of LIVE view.
    live_robot.set_data(
        [
            0.0
        ],
        [
            0.0
        ]
    )

    # LIVE LiDAR is intentionally ROBOT-RELATIVE.
    #
    # The robot is always at the centre and its physical front
    # is always +X / RIGHT in this panel.
    #
    # Do NOT rotate this panel by the world/navX heading.
    # Rotating it mixes coordinate frames and makes a normal
    # heading reset look wrong.
    live_heading_line.set_data(
        [
            0.0,
            0.35
        ],
        [
            0.0,
            0.0
        ]
    )

    # --------------------------------------------------------
    # OCCUPANCY MAP + FUSED ROUTE
    # --------------------------------------------------------

    map_image.set_data(
        get_occupancy_image()
    )

    route_line.set_data(
        route_x,
        route_y
    )

    if fusion_pose_ok:

        map_robot.set_data(
            [
                display_robot_x
            ],
            [
                display_robot_y
            ]
        )

        (
            heading_dx,
            heading_dy
        ) = display_heading_vector(
            display_robot_heading,
            HEADING_ARROW_LENGTH_M
        )

        heading_arrow.set_positions(
            (
                display_robot_x,
                display_robot_y
            ),
            (
                display_robot_x
                +
                heading_dx,
                display_robot_y
                +
                heading_dy
            )
        )

    # --------------------------------------------------------
    # INFORMATION PANEL
    # --------------------------------------------------------

    if fusion_reset_pending:

        fused_status_label.config(
            text="FUSION: RESETTING"
        )

    else:

        fused_status_label.config(
            text=(
                "FUSION: ACTIVE"
                if fusion_pose_ok
                else
                "FUSION: WAITING"
            )
        )

    pose_x_label.config(
        text=(
            f"Fused X: "
            f"{display_robot_x:.3f} m"
        )
    )

    pose_y_label.config(
        text=(
            f"Fused Y: "
            f"{display_robot_y:.3f} m"
        )
    )

    pose_heading_label.config(
        text=(
            f"Heading: "
            f"{display_robot_heading:.2f}°"
        )
    )

    lidar_state_label.config(
        text=(
            f"LiDAR ICP: "
            f"{localization_state}"
        )
    )

    lidar_quality_label.config(
        text=(
            f"ICP error: "
            f"{last_icp_error:.3f} m\n"
            f"Inliers: "
            f"{last_inlier_ratio:.2f}"
        )
    )

    obstacle_label.config(
        text=(
            f"Nearest obstacle: "
            f"{nearest_obstacle_distance:.2f} m\n"
            f"LiDAR angle: "
            f"{nearest_obstacle_angle:.1f}°"
        )
    )

    correction_error = fusion_table.getNumber(
        "CorrectionErrorM",
        0.0
    )

    correction_applied = fusion_table.getNumber(
        "CorrectionAppliedM",
        0.0
    )

    fusion_correction_label.config(
        text=(
            f"Fusion correction error: "
            f"{correction_error:.3f} m\n"
            f"Applied this update: "
            f"{correction_applied:.3f} m"
        )
    )

    if fusion_pose_ok and map_pose_stable:

        mapping_state_label.config(
            text="MAP: UPDATING"
        )

    elif fusion_pose_ok:

        mapping_state_label.config(
            text="MAP: PAUSED - FUSION CORRECTING"
        )

    else:

        mapping_state_label.config(
            text="MAP: WAITING FOR FUSION"
        )

    canvas.draw_idle()

    root.after(
        DISPLAY_UPDATE_MS,
        tracking_update
    )


# ============================================================
# KEY EVENTS
# ============================================================

def on_key_press(
        event):

    global space_stop_active

    key = event.keysym.lower()

    if key == "escape":

        close_program()

        return

    if key == "p":

        save_csv()

        return

    if key == "m":

        save_map()

        return

    if key == "c":

        reset_display_and_map()

        return

    if key == "r":

        reset_lidar_localization()

        return

    if key == "space":

        space_stop_active = True

        pressed_keys.clear()

        send_stop()

        drive_status_label.config(
            text="STOPPED - SPACE"
        )

        return

    if space_stop_active:
        return

    if key in (
        "w",
        "a",
        "s",
        "d",
        "q",
        "e"
    ):

        pressed_keys.add(
            key
        )


def on_key_release(
        event):

    global space_stop_active

    key = event.keysym.lower()

    if key == "space":

        space_stop_active = False

        pressed_keys.clear()

        send_stop()

        drive_status_label.config(
            text="READY"
        )

        return

    if key in pressed_keys:

        pressed_keys.remove(
            key
        )


def on_window_click(
        event):

    try:
        root.focus_force()

    except tk.TclError:
        pass


# ============================================================
# CLOSE
# ============================================================

def close_program():

    global program_running

    program_running = False

    pressed_keys.clear()

    keyboard_table.putBoolean(
        "Enabled",
        False
    )

    send_stop()

    try:
        root.destroy()

    except tk.TclError:
        pass


# ============================================================
# GUI
# ============================================================

root = tk.Tk()

root.title(
    "Training Robot - Fusion Localization + Live LiDAR"
)

root.geometry(
    "1550x780"
)

root.minsize(
    1300,
    700
)

root.protocol(
    "WM_DELETE_WINDOW",
    close_program
)

root.columnconfigure(
    0,
    weight=0
)

root.columnconfigure(
    1,
    weight=1
)

root.rowconfigure(
    0,
    weight=1
)


# ============================================================
# LEFT CONTROL PANEL
# ============================================================

control_frame = ttk.Frame(
    root,
    padding=12
)

control_frame.grid(
    row=0,
    column=0,
    sticky="nsew"
)


title_label = ttk.Label(
    control_frame,
    text="FUSION LOCALIZATION",
    font=(
        "Arial",
        17,
        "bold"
    )
)

title_label.pack(
    pady=8
)


controls_label = ttk.Label(
    control_frame,
    text=(
        "W = Forward\n"
        "S = Backward\n"
        "A = Crab Left\n"
        "D = Crab Right\n"
        "Q = Rotate Left\n"
        "E = Rotate Right\n\n"
        "SPACE = Stop immediately\n"
        "C = FULL RESET fusion + LiDAR + map\n"
        "R = Reset LiDAR ICP only\n"
        "P = Save route CSV\n"
        "M = Save occupancy map\n"
        "ESC = Close"
    ),
    justify="left"
)

controls_label.pack(
    pady=8
)


ttk.Separator(
    control_frame,
    orient="horizontal"
).pack(
    fill="x",
    pady=8
)


drive_status_label = ttk.Label(
    control_frame,
    text="CLICK WINDOW TO DRIVE",
    font=(
        "Arial",
        11,
        "bold"
    )
)

drive_status_label.pack(
    pady=5
)


x_command_label = ttk.Label(
    control_frame,
    text="X Cmd: 0.00"
)

x_command_label.pack()


y_command_label = ttk.Label(
    control_frame,
    text="Y Cmd: 0.00"
)

y_command_label.pack()


z_command_label = ttk.Label(
    control_frame,
    text="Z Cmd: 0.00"
)

z_command_label.pack()


ttk.Separator(
    control_frame,
    orient="horizontal"
).pack(
    fill="x",
    pady=8
)


fused_status_label = ttk.Label(
    control_frame,
    text="FUSION: WAITING",
    font=(
        "Arial",
        11,
        "bold"
    )
)

fused_status_label.pack(
    pady=4
)


pose_x_label = ttk.Label(
    control_frame,
    text="Fused X: 0.000 m"
)

pose_x_label.pack()


pose_y_label = ttk.Label(
    control_frame,
    text="Fused Y: 0.000 m"
)

pose_y_label.pack()


pose_heading_label = ttk.Label(
    control_frame,
    text="Heading: 0.00°"
)

pose_heading_label.pack()


heading_help_label = ttk.Label(
    control_frame,
    text=(
        "Left arrow = robot front in LOCAL LiDAR view\n"
        "Right arrow = same front direction in WORLD map"
    ),
    justify="center"
)

heading_help_label.pack(
    pady=4
)


ttk.Separator(
    control_frame,
    orient="horizontal"
).pack(
    fill="x",
    pady=8
)


lidar_state_label = ttk.Label(
    control_frame,
    text="LiDAR ICP: STARTING"
)

lidar_state_label.pack()


lidar_quality_label = ttk.Label(
    control_frame,
    text=(
        "ICP error: 999.000 m\n"
        "Inliers: 0.00"
    )
)

lidar_quality_label.pack()


obstacle_label = ttk.Label(
    control_frame,
    text=(
        "Nearest obstacle: --\n"
        "LiDAR angle: --"
    )
)

obstacle_label.pack(
    pady=5
)


fusion_correction_label = ttk.Label(
    control_frame,
    text=(
        "Fusion correction error: 0.000 m\n"
        "Applied this update: 0.000 m"
    )
)

fusion_correction_label.pack(
    pady=5
)


mapping_state_label = ttk.Label(
    control_frame,
    text="MAP: WAITING FOR FUSION",
    font=(
        "Arial",
        9,
        "bold"
    )
)

mapping_state_label.pack(
    pady=3
)


save_status_label = ttk.Label(
    control_frame,
    text="No file saved.",
    wraplength=230,
    justify="center"
)

save_status_label.pack(
    pady=8
)


# ============================================================
# MATPLOTLIB AREA
# ============================================================

plot_frame = ttk.Frame(
    root,
    padding=5
)

plot_frame.grid(
    row=0,
    column=1,
    sticky="nsew"
)

plot_frame.columnconfigure(
    0,
    weight=1
)

plot_frame.rowconfigure(
    0,
    weight=1
)


figure = Figure(
    figsize=(
        11.5,
        7.0
    ),
    dpi=100
)


live_axis = figure.add_subplot(
    121
)

map_axis = figure.add_subplot(
    122
)


# ============================================================
# LIVE LIDAR
# ============================================================

live_axis.set_title(
    "LIVE LiDAR - Robot Frame (Forward = Right)"
)

live_axis.set_xlabel(
    "Forward / Backward (m)"
)

live_axis.set_ylabel(
    "Left / Right (m)"
)

live_axis.set_aspect(
    "equal",
    adjustable="box"
)

live_axis.set_xlim(
    -LIVE_HALF_RANGE_M,
    LIVE_HALF_RANGE_M
)

live_axis.set_ylim(
    -LIVE_HALF_RANGE_M,
    LIVE_HALF_RANGE_M
)

live_axis.grid(
    True
)


live_scatter = live_axis.scatter(
    [],
    [],
    s=LIVE_POINT_SIZE,
    marker="s"
)


live_robot, = live_axis.plot(
    [],
    [],
    marker="o",
    linestyle="None",
    markersize=8
)


live_heading_line, = live_axis.plot(
    [],
    [],
    linewidth=2
)


# ============================================================
# FUSED OCCUPANCY MAP
# ============================================================

map_axis.set_title(
    "FUSED World Route + LiDAR Occupancy Map"
)

map_axis.set_xlabel(
    "Display X (m)"
)

map_axis.set_ylabel(
    "Display Y (m)"
)

map_axis.set_aspect(
    "equal",
    adjustable="box"
)

half_map = (
    MAP_SIZE_M
    /
    2.0
)

map_axis.set_xlim(
    -half_map,
    half_map
)

map_axis.set_ylim(
    -half_map,
    half_map
)

map_axis.grid(
    True
)


map_image = map_axis.imshow(
    get_occupancy_image(),
    origin="lower",
    extent=[
        -half_map,
        half_map,
        -half_map,
        half_map
    ],
    vmin=0.0,
    vmax=1.0,
    interpolation="nearest"
)


route_line, = map_axis.plot(
    route_x,
    route_y,
    linewidth=2,
    label="Fused route"
)


map_robot, = map_axis.plot(
    [
        0.0
    ],
    [
        0.0
    ],
    marker="o",
    linestyle="None",
    markersize=8,
    label="Robot"
)


heading_arrow = FancyArrowPatch(
    (
        0.0,
        0.0
    ),
    (
        HEADING_ARROW_LENGTH_M,
        0.0
    ),
    arrowstyle="-|>",
    mutation_scale=16,
    linewidth=2
)

map_axis.add_patch(
    heading_arrow
)

map_axis.legend(
    loc="upper right"
)


canvas = FigureCanvasTkAgg(
    figure,
    master=plot_frame
)

canvas.get_tk_widget().grid(
    row=0,
    column=0,
    sticky="nsew"
)


# ============================================================
# LOG TABLE
# ============================================================

log_title = ttk.Label(
    control_frame,
    text="Latest Fused Pose",
    font=(
        "Arial",
        10,
        "bold"
    )
)

log_title.pack(
    pady=4
)


columns = (
    "sample",
    "time",
    "x",
    "y",
    "heading",
    "lidar"
)


log_table = ttk.Treeview(
    control_frame,
    columns=columns,
    show="headings",
    height=8
)

log_table.heading(
    "sample",
    text="#"
)

log_table.heading(
    "time",
    text="t"
)

log_table.heading(
    "x",
    text="X"
)

log_table.heading(
    "y",
    text="Y"
)

log_table.heading(
    "heading",
    text="°"
)

log_table.heading(
    "lidar",
    text="ICP"
)

log_table.column(
    "sample",
    width=32,
    anchor="center"
)

log_table.column(
    "time",
    width=45,
    anchor="center"
)

log_table.column(
    "x",
    width=55,
    anchor="center"
)

log_table.column(
    "y",
    width=55,
    anchor="center"
)

log_table.column(
    "heading",
    width=45,
    anchor="center"
)

log_table.column(
    "lidar",
    width=42,
    anchor="center"
)

log_table.pack(
    fill="x",
    pady=4
)


# ============================================================
# EVENTS
# ============================================================

root.bind(
    "<KeyPress>",
    on_key_press
)

root.bind(
    "<KeyRelease>",
    on_key_release
)

root.bind(
    "<Button-1>",
    on_window_click
)


# ============================================================
# SAFE START
# ============================================================

keyboard_table.putBoolean(
    "Enabled",
    False
)

keyboard_table.putNumber(
    "X",
    0.0
)

keyboard_table.putNumber(
    "Y",
    0.0
)

keyboard_table.putNumber(
    "Z",
    0.0
)

keyboard_table.putNumber(
    "Heartbeat",
    0.0
)

reset_lidar_localization()


# ============================================================
# START
# ============================================================

root.after(
    DRIVE_UPDATE_MS,
    update_drive
)

root.after(
    DISPLAY_UPDATE_MS,
    tracking_update
)


try:

    root.mainloop()

finally:

    keyboard_table.putBoolean(
        "Enabled",
        False
    )

    send_stop()
