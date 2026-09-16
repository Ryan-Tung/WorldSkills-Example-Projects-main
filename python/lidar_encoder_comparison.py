import os
import csv
import time
import math
import tkinter as tk
from tkinter import ttk

import numpy as np

from scipy.spatial import cKDTree

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import FancyArrowPatch

from networktables import NetworkTables


# ============================================================
# ROBOT CONNECTION
# ============================================================

ROBOT_IP = "10.23.45.2"

NetworkTables.initialize(
    server=ROBOT_IP
)

NetworkTables.setUpdateRate(
    0.010
)


# ============================================================
# NETWORKTABLES
# ============================================================

# Python -> Java KeyboardDrive command
keyboard_table = NetworkTables.getTable(
    "KeyboardDrive"
)

# Raw LiDAR scan from LidarTest.java
lidar_table = NetworkTables.getTable(
    "Lidar"
)

# navX heading from DriveTrain.java
robot_pose_table = NetworkTables.getTable(
    "RobotPose"
)

# Raw wheel encoder values already shown in Shuffleboard
shuffleboard_table = NetworkTables.getTable(
    "Shuffleboard"
)

training_robot_table = shuffleboard_table.getSubTable(
    "Training Robot"
)


# ============================================================
# SAVE FOLDER
# ============================================================

SCRIPT_FOLDER = os.path.dirname(
    os.path.abspath(__file__)
)


# ============================================================
# KEYBOARD SETTINGS
#
# Same controls as your previous keyboard modes.
# ============================================================

MOVE_SPEED = 0.25
TURN_SPEED = 0.15

DRIVE_UPDATE_MS = 50

# Run both localization methods at the SAME update rate so the
# comparison is fair.
TRACK_UPDATE_MS = 100

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

LIVE_LIDAR_HALF_RANGE_M = 5.0
LIVE_POINT_SIZE = 8


# ============================================================
# LiDAR ICP SETTINGS
#
# navX supplies rotation.
# ICP estimates translation only.
# ============================================================

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

LIDAR_STATIONARY_TRANSLATION_M = 0.006


# ============================================================
# ENCODER SETTINGS
#
# Encoder distance published by your robot is in mm.
# ============================================================

ENCODER_TO_METERS = 1.0 / 1000.0

ENCODER_DELTA_DEADBAND_M = 0.0005

# Protect against an impossible one-update encoder jump.
MAX_ENCODER_STEP_M = 0.15


# ============================================================
# COMPARISON RULES
#
# IMPORTANT:
#
# BOTH localization methods receive the same movement rules.
#
# W/S/A/D:
#   X/Y tracking enabled.
#
# W+Q / W+E / A+Q etc:
#   X/Y tracking enabled + heading changes.
#
# Q/E only:
#   X/Y LOCKED for BOTH methods.
#   Only heading changes.
#
# No movement command:
#   X/Y LOCKED for BOTH methods.
#
# This means we compare the sensors fairly using exactly the
# same physical drive command.
# ============================================================


# ============================================================
# DISPLAY SETTINGS
# ============================================================

INITIAL_ROUTE_HALF_RANGE_M = 1.5
ROUTE_EXPAND_MARGIN_M = 0.50

HEADING_ARROW_LENGTH_M = 0.25

MAX_VISIBLE_LOG_ROWS = 12


# ============================================================
# PROGRAM STATE
# ============================================================

program_running = True
space_stop_active = False
pressed_keys = set()

heartbeat = 0

current_command_x = 0.0
current_command_y = 0.0
current_command_z = 0.0


# ============================================================
# SHARED HEADING
#
# Both LiDAR and Encoder localization use the SAME navX
# heading and the SAME zero reference.
# ============================================================

heading_zero_initialized = False
heading_zero_raw = 0.0

current_heading = 0.0
previous_heading = 0.0


# ============================================================
# LIDAR LOCALIZATION STATE
#
# Internal world frame:
#
# +X = robot-right direction at reset
# +Y = robot-forward direction at reset
# ============================================================

lidar_x = 0.0
lidar_y = 0.0

previous_lidar_scan = None

keyframe_scan = None
keyframe_x = 0.0
keyframe_y = 0.0
keyframe_heading = 0.0

lidar_valid = False
lidar_state = "WAITING"

last_icp_error = 999.0
last_icp_inlier_ratio = 0.0
last_lidar_translation_m = 0.0


# ============================================================
# ENCODER LOCALIZATION STATE
#
# Same internal world frame as LiDAR:
#
# +X = reset-right
# +Y = reset-forward
# ============================================================

encoder_x = 0.0
encoder_y = 0.0

previous_left_encoder = 0.0
previous_right_encoder = 0.0
previous_back_encoder = 0.0

encoder_reference_initialized = False

last_encoder_translation_m = 0.0
last_encoder_step_accepted = False


# ============================================================
# DISPLAY ROUTES
#
# Display convention:
#
#                   -90 / 270°
#                       ↑
#                       |
# 180°  <---------------+---------------> 0° / +X
#                       |
#                       ↓
#                      90°
#
# Reset-forward is shown to the RIGHT.
# ============================================================

lidar_route_x = [0.0]
lidar_route_y = [0.0]

encoder_route_x = [0.0]
encoder_route_y = [0.0]

route_min_x = -INITIAL_ROUTE_HALF_RANGE_M
route_max_x = INITIAL_ROUTE_HALF_RANGE_M
route_min_y = -INITIAL_ROUTE_HALF_RANGE_M
route_max_y = INITIAL_ROUTE_HALF_RANGE_M


# ============================================================
# COMPARISON LOG
# ============================================================

comparison_rows = []
sample_counter = 0

difference_samples = []

test_start_time = time.monotonic()


# ============================================================
# GENERAL HELPERS
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
# COMMAND STATE
# ============================================================

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


def any_motion_commanded():

    return (
        translation_commanded()
        or
        rotation_commanded()
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
            text="ROTATING - XY LOCKED"
        )

    else:

        drive_status_label.config(
            text="READY - XY LOCKED"
        )

    root.after(
        DRIVE_UPDATE_MS,
        update_drive
    )


# ============================================================
# SHARED NAVX HEADING
# ============================================================

def read_raw_heading():

    value = robot_pose_table.getNumber(
        "Heading",
        999999.0
    )

    if value == 999999.0:
        return None

    return float(
        value
    )


def update_shared_heading():

    global heading_zero_initialized
    global heading_zero_raw

    global current_heading

    raw_heading = read_raw_heading()

    if raw_heading is None:
        return False

    if not heading_zero_initialized:

        heading_zero_raw = (
            raw_heading
        )

        heading_zero_initialized = True

    current_heading = normalize_heading(
        raw_heading
        -
        heading_zero_raw
    )

    return True


# ============================================================
# RAW ENCODER READ
# ============================================================

def read_raw_encoders():

    left = training_robot_table.getNumber(
        "Left Encoder",
        999999.0
    )

    right = training_robot_table.getNumber(
        "Right Encoder",
        999999.0
    )

    back = training_robot_table.getNumber(
        "Back Encoder",
        999999.0
    )

    if (
        left == 999999.0
        or
        right == 999999.0
        or
        back == 999999.0
    ):

        return None

    return (
        float(left),
        float(right),
        float(back)
    )


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
            (0, 2),
            dtype=float
        )

    angle_rad = np.radians(
        angles
    )

    distance_m = (
        distances_mm
        /
        1000.0
    )

    # LiDAR / robot local frame:
    #
    # 0°   -> +Y / forward
    # 90°  -> +X / right

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
# ICP HELPERS
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
        maximum_points):

    if len(points) <= maximum_points:
        return points.copy()

    indices = np.linspace(
        0,
        len(points) - 1,
        maximum_points,
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
            np.zeros(
                2,
                dtype=float
            ),
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
                np.zeros(
                    2,
                    dtype=float
                ),
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
                np.zeros(
                    2,
                    dtype=float
                ),
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
            np.zeros(
                2,
                dtype=float
            ),
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
# LiDAR KEYFRAME
# ============================================================

def create_lidar_keyframe(
        current_scan):

    global keyframe_scan
    global keyframe_x
    global keyframe_y
    global keyframe_heading

    keyframe_scan = (
        current_scan.copy()
    )

    keyframe_x = (
        lidar_x
    )

    keyframe_y = (
        lidar_y
    )

    keyframe_heading = (
        current_heading
    )


def lidar_keyframe_needed():

    if keyframe_scan is None:
        return True

    movement = math.hypot(
        lidar_x
        -
        keyframe_x,

        lidar_y
        -
        keyframe_y
    )

    heading_change = abs(
        heading_difference(
            current_heading,
            keyframe_heading
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
# UPDATE LiDAR-ONLY LOCALIZATION
# ============================================================

def update_lidar_localization(
        current_scan,
        heading_before_update):

    global lidar_x
    global lidar_y

    global previous_lidar_scan

    global lidar_valid
    global lidar_state

    global last_icp_error
    global last_icp_inlier_ratio
    global last_lidar_translation_m

    if len(current_scan) < MIN_SCAN_POINTS:

        lidar_valid = False

        lidar_state = (
            "NOT_ENOUGH_POINTS"
        )

        return

    # --------------------------------------------------------
    # FIRST SCAN
    # --------------------------------------------------------

    if previous_lidar_scan is None:

        previous_lidar_scan = (
            current_scan.copy()
        )

        create_lidar_keyframe(
            current_scan
        )

        lidar_valid = True

        lidar_state = (
            "INITIALIZED"
        )

        last_lidar_translation_m = (
            0.0
        )

        return

    # --------------------------------------------------------
    # FAIR COMPARISON RULE:
    #
    # No translation command means both systems hold X/Y.
    #
    # This includes:
    # - Q/E only
    # - no key pressed
    #
    # LiDAR reference is refreshed so the next translation
    # starts from a fresh scan.
    # --------------------------------------------------------

    if not translation_commanded():

        previous_lidar_scan = (
            current_scan.copy()
        )

        lidar_valid = True

        last_lidar_translation_m = (
            0.0
        )

        if pure_rotation_commanded():

            lidar_state = (
                "ROTATION_XY_HOLD"
            )

        else:

            lidar_state = (
                "IDLE_XY_HOLD"
            )

        if lidar_keyframe_needed():

            create_lidar_keyframe(
                current_scan
            )

        return

    # --------------------------------------------------------
    # NORMAL TRANSLATION MATCH
    #
    # This also handles W+Q / W+E because translation is
    # commanded.
    # --------------------------------------------------------

    heading_delta = heading_difference(
        current_heading,
        heading_before_update
    )

    (
        relative_translation,
        matched,
        error,
        inlier_ratio
    ) = translation_icp(
        current_scan,
        previous_lidar_scan,
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

        keyframe_heading_delta = heading_difference(
            current_heading,
            keyframe_heading
        )

        (
            recovery_translation,
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
                recovery_translation,
                recovery_error,
                recovery_ratio,
                recovery=True
            )
        ):

            local_robot_movement = (
                -recovery_translation
            )

            world_movement = (
                local_translation_to_world(
                    local_robot_movement,
                    keyframe_heading
                )
            )

            lidar_x = (
                keyframe_x
                +
                world_movement[0]
            )

            lidar_y = (
                keyframe_y
                +
                world_movement[1]
            )

            relative_translation = (
                recovery_translation
            )

            error = (
                recovery_error
            )

            inlier_ratio = (
                recovery_ratio
            )

            accepted = True
            used_recovery = True

    # --------------------------------------------------------
    # ACCEPT NORMAL ICP
    # --------------------------------------------------------

    if accepted:

        if not used_recovery:

            # ICP translation is the amount needed to shift the
            # CURRENT scan back onto the previous scan.
            #
            # Robot motion is therefore the opposite.
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
                LIDAR_STATIONARY_TRANSLATION_M
            ):

                local_robot_movement = (
                    np.zeros(
                        2,
                        dtype=float
                    )
                )

            # Use previous / interval-start heading to transform
            # this scan interval into the shared world frame.
            world_movement = (
                local_translation_to_world(
                    local_robot_movement,
                    heading_before_update
                )
            )

            lidar_x += (
                world_movement[0]
            )

            lidar_y += (
                world_movement[1]
            )

        last_icp_error = (
            error
        )

        last_icp_inlier_ratio = (
            inlier_ratio
        )

        last_lidar_translation_m = float(
            np.linalg.norm(
                relative_translation
            )
        )

        lidar_valid = True

        if used_recovery:

            lidar_state = (
                "RECOVERY"
            )

        elif rotation_commanded():

            lidar_state = (
                "TRACKING_TURN"
            )

        else:

            lidar_state = (
                "TRACKING"
            )

        previous_lidar_scan = (
            current_scan.copy()
        )

        if lidar_keyframe_needed():

            create_lidar_keyframe(
                current_scan
            )

    else:

        # Hold position on poor ICP.
        lidar_valid = False

        lidar_state = (
            "HOLD_BAD_ICP"
        )

        previous_lidar_scan = (
            current_scan.copy()
        )

        last_lidar_translation_m = (
            0.0
        )


# ============================================================
# ENCODER REFERENCE
# ============================================================

def initialize_encoder_reference():

    global encoder_reference_initialized

    global previous_left_encoder
    global previous_right_encoder
    global previous_back_encoder

    values = read_raw_encoders()

    if values is None:
        return False

    (
        previous_left_encoder,
        previous_right_encoder,
        previous_back_encoder
    ) = values

    encoder_reference_initialized = True

    return True


# ============================================================
# UPDATE ENCODER-ONLY LOCALIZATION
# ============================================================

def update_encoder_localization(
        heading_before_update):

    global encoder_x
    global encoder_y

    global previous_left_encoder
    global previous_right_encoder
    global previous_back_encoder

    global encoder_reference_initialized

    global last_encoder_translation_m
    global last_encoder_step_accepted

    values = read_raw_encoders()

    if values is None:

        last_encoder_step_accepted = (
            False
        )

        return False

    (
        current_left,
        current_right,
        current_back
    ) = values

    if not encoder_reference_initialized:

        previous_left_encoder = (
            current_left
        )

        previous_right_encoder = (
            current_right
        )

        previous_back_encoder = (
            current_back
        )

        encoder_reference_initialized = (
            True
        )

        last_encoder_step_accepted = (
            False
        )

        return True

    delta_left = (
        current_left
        -
        previous_left_encoder
    )

    delta_right = (
        current_right
        -
        previous_right_encoder
    )

    delta_back = (
        current_back
        -
        previous_back_encoder
    )

    # Always consume current values.
    #
    # Therefore Q/E rotation encoder activity can NEVER become
    # a jump when translation starts later.
    previous_left_encoder = (
        current_left
    )

    previous_right_encoder = (
        current_right
    )

    previous_back_encoder = (
        current_back
    )

    # --------------------------------------------------------
    # FAIR COMPARISON RULE:
    #
    # Q/E only or idle:
    # X/Y HOLD.
    # --------------------------------------------------------

    if not translation_commanded():

        last_encoder_translation_m = (
            0.0
        )

        last_encoder_step_accepted = (
            False
        )

        return True

    # --------------------------------------------------------
    # 3-WHEEL HOLONOMIC ODOMETRY
    # --------------------------------------------------------

    local_y_mm = (
        delta_left
        -
        delta_right
    ) / 2.0

    local_x_mm = (
        (
            delta_left
            +
            delta_right
        )
        /
        (
            2.0
            *
            math.sqrt(3.0)
        )
    ) - delta_back

    local_x = (
        local_x_mm
        *
        ENCODER_TO_METERS
    )

    local_y = (
        local_y_mm
        *
        ENCODER_TO_METERS
    )

    movement_m = math.hypot(
        local_x,
        local_y
    )

    last_encoder_translation_m = (
        movement_m
    )

    if movement_m < ENCODER_DELTA_DEADBAND_M:

        last_encoder_step_accepted = (
            False
        )

        return True

    if movement_m > MAX_ENCODER_STEP_M:

        last_encoder_step_accepted = (
            False
        )

        return True

    # --------------------------------------------------------
    # MIDPOINT HEADING
    #
    # Makes W+Q / W+E comparison more accurate.
    # --------------------------------------------------------

    heading_delta = heading_difference(
        current_heading,
        heading_before_update
    )

    midpoint_heading = normalize_heading(
        heading_before_update
        +
        heading_delta
        *
        0.5
    )

    heading_rad = math.radians(
        midpoint_heading
    )

    world_delta_x = (
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

    world_delta_y = (
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

    encoder_x += (
        world_delta_x
    )

    encoder_y += (
        world_delta_y
    )

    last_encoder_step_accepted = (
        True
    )

    return True


# ============================================================
# WORLD -> DISPLAY
# ============================================================

def world_to_display(
        world_x,
        world_y):

    display_x = (
        world_y
    )

    display_y = (
        -world_x
    )

    return (
        display_x,
        display_y
    )


def display_heading_vector(
        heading_deg,
        length):

    heading_rad = math.radians(
        heading_deg
    )

    dx = (
        length
        *
        math.cos(
            heading_rad
        )
    )

    dy = (
        -length
        *
        math.sin(
            heading_rad
        )
    )

    return (
        dx,
        dy
    )


# ============================================================
# RESET BOTH LOCALIZATION METHODS TOGETHER
# ============================================================

def reset_comparison():

    global heading_zero_initialized
    global heading_zero_raw

    global current_heading
    global previous_heading

    global lidar_x
    global lidar_y
    global previous_lidar_scan

    global keyframe_scan
    global keyframe_x
    global keyframe_y
    global keyframe_heading

    global lidar_valid
    global lidar_state

    global encoder_x
    global encoder_y

    global encoder_reference_initialized
    global previous_left_encoder
    global previous_right_encoder
    global previous_back_encoder

    global lidar_route_x
    global lidar_route_y

    global encoder_route_x
    global encoder_route_y

    global comparison_rows
    global sample_counter
    global difference_samples
    global test_start_time

    global route_min_x
    global route_max_x
    global route_min_y
    global route_max_y

    pressed_keys.clear()

    send_stop()

    # --------------------------------------------------------
    # ONE SHARED HEADING ZERO
    # --------------------------------------------------------

    raw_heading = read_raw_heading()

    if raw_heading is None:

        reset_status_label.config(
            text="RESET FAILED: navX unavailable."
        )

        return

    heading_zero_raw = (
        raw_heading
    )

    heading_zero_initialized = (
        True
    )

    current_heading = 0.0
    previous_heading = 0.0

    # --------------------------------------------------------
    # ENCODER BASELINE
    # --------------------------------------------------------

    encoder_values = read_raw_encoders()

    if encoder_values is None:

        reset_status_label.config(
            text="RESET FAILED: encoder data unavailable."
        )

        return

    (
        previous_left_encoder,
        previous_right_encoder,
        previous_back_encoder
    ) = encoder_values

    encoder_reference_initialized = (
        True
    )

    encoder_x = 0.0
    encoder_y = 0.0

    # --------------------------------------------------------
    # LIDAR BASELINE
    # --------------------------------------------------------

    (
        angles,
        distances
    ) = get_full_scan()

    current_scan = scan_to_xy(
        angles,
        distances
    )

    lidar_x = 0.0
    lidar_y = 0.0

    if len(current_scan) >= MIN_SCAN_POINTS:

        previous_lidar_scan = (
            current_scan.copy()
        )

        lidar_valid = True

        lidar_state = (
            "RESET_READY"
        )

        keyframe_scan = (
            current_scan.copy()
        )

        keyframe_x = 0.0
        keyframe_y = 0.0
        keyframe_heading = 0.0

    else:

        previous_lidar_scan = None

        keyframe_scan = None

        lidar_valid = False

        lidar_state = (
            "RESET_WAITING_FOR_SCAN"
        )

    # --------------------------------------------------------
    # CLEAR DISPLAY / LOG
    # --------------------------------------------------------

    lidar_route_x = [
        0.0
    ]

    lidar_route_y = [
        0.0
    ]

    encoder_route_x = [
        0.0
    ]

    encoder_route_y = [
        0.0
    ]

    comparison_rows = []

    difference_samples = []

    sample_counter = 0

    test_start_time = (
        time.monotonic()
    )

    for row_id in log_table.get_children():

        log_table.delete(
            row_id
        )

    route_min_x = (
        -INITIAL_ROUTE_HALF_RANGE_M
    )

    route_max_x = (
        INITIAL_ROUTE_HALF_RANGE_M
    )

    route_min_y = (
        -INITIAL_ROUTE_HALF_RANGE_M
    )

    route_max_y = (
        INITIAL_ROUTE_HALF_RANGE_M
    )

    comparison_axis.set_xlim(
        route_min_x,
        route_max_x
    )

    comparison_axis.set_ylim(
        route_min_y,
        route_max_y
    )

    reset_status_label.config(
        text=(
            "RESET COMPLETE\n"
            "LiDAR + Encoder + navX = 0,0,0"
        )
    )

    update_comparison_artists()

    canvas.draw_idle()


# ============================================================
# ROUTE LIMITS
# ============================================================

def update_route_limits(
        lidar_display_x,
        lidar_display_y,
        encoder_display_x,
        encoder_display_y):

    global route_min_x
    global route_max_x
    global route_min_y
    global route_max_y

    changed = False

    min_current_x = min(
        lidar_display_x,
        encoder_display_x
    )

    max_current_x = max(
        lidar_display_x,
        encoder_display_x
    )

    min_current_y = min(
        lidar_display_y,
        encoder_display_y
    )

    max_current_y = max(
        lidar_display_y,
        encoder_display_y
    )

    if (
        min_current_x
        <
        route_min_x
        +
        ROUTE_EXPAND_MARGIN_M
    ):

        route_min_x = (
            min_current_x
            -
            ROUTE_EXPAND_MARGIN_M
        )

        changed = True

    if (
        max_current_x
        >
        route_max_x
        -
        ROUTE_EXPAND_MARGIN_M
    ):

        route_max_x = (
            max_current_x
            +
            ROUTE_EXPAND_MARGIN_M
        )

        changed = True

    if (
        min_current_y
        <
        route_min_y
        +
        ROUTE_EXPAND_MARGIN_M
    ):

        route_min_y = (
            min_current_y
            -
            ROUTE_EXPAND_MARGIN_M
        )

        changed = True

    if (
        max_current_y
        >
        route_max_y
        -
        ROUTE_EXPAND_MARGIN_M
    ):

        route_max_y = (
            max_current_y
            +
            ROUTE_EXPAND_MARGIN_M
        )

        changed = True

    if changed:

        comparison_axis.set_xlim(
            route_min_x,
            route_max_x
        )

        comparison_axis.set_ylim(
            route_min_y,
            route_max_y
        )


# ============================================================
# LOG ONE COMPARISON SAMPLE
# ============================================================

def add_comparison_sample():

    global sample_counter

    (
        lidar_display_x,
        lidar_display_y
    ) = world_to_display(
        lidar_x,
        lidar_y
    )

    (
        encoder_display_x,
        encoder_display_y
    ) = world_to_display(
        encoder_x,
        encoder_y
    )

    difference_x = (
        lidar_display_x
        -
        encoder_display_x
    )

    difference_y = (
        lidar_display_y
        -
        encoder_display_y
    )

    difference_distance = math.hypot(
        difference_x,
        difference_y
    )

    difference_samples.append(
        difference_distance
    )

    sample_counter += 1

    elapsed = (
        time.monotonic()
        -
        test_start_time
    )

    row = {
        "sample": sample_counter,
        "elapsed": elapsed,

        "lidar_x": lidar_display_x,
        "lidar_y": lidar_display_y,

        "encoder_x": encoder_display_x,
        "encoder_y": encoder_display_y,

        "heading": current_heading,

        "difference_x": difference_x,
        "difference_y": difference_y,
        "difference_distance": difference_distance,

        "lidar_valid": lidar_valid,
        "lidar_state": lidar_state,

        "icp_error": last_icp_error,
        "icp_inlier_ratio": last_icp_inlier_ratio,

        "cmd_x": current_command_x,
        "cmd_y": current_command_y,
        "cmd_z": current_command_z
    }

    comparison_rows.append(
        row
    )

    lidar_route_x.append(
        lidar_display_x
    )

    lidar_route_y.append(
        lidar_display_y
    )

    encoder_route_x.append(
        encoder_display_x
    )

    encoder_route_y.append(
        encoder_display_y
    )

    log_table.insert(
        "",
        "end",
        values=(
            row[
                "sample"
            ],
            f'{row["lidar_x"]:.3f}',
            f'{row["lidar_y"]:.3f}',
            f'{row["encoder_x"]:.3f}',
            f'{row["encoder_y"]:.3f}',
            f'{row["difference_distance"]:.3f}'
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


# ============================================================
# COMPARISON DISPLAY
# ============================================================

def update_comparison_artists():

    (
        lidar_display_x,
        lidar_display_y
    ) = world_to_display(
        lidar_x,
        lidar_y
    )

    (
        encoder_display_x,
        encoder_display_y
    ) = world_to_display(
        encoder_x,
        encoder_y
    )

    lidar_route_line.set_data(
        lidar_route_x,
        lidar_route_y
    )

    encoder_route_line.set_data(
        encoder_route_x,
        encoder_route_y
    )

    lidar_marker.set_data(
        [
            lidar_display_x
        ],
        [
            lidar_display_y
        ]
    )

    encoder_marker.set_data(
        [
            encoder_display_x
        ],
        [
            encoder_display_y
        ]
    )

    (
        heading_dx,
        heading_dy
    ) = display_heading_vector(
        current_heading,
        HEADING_ARROW_LENGTH_M
    )

    lidar_heading_arrow.set_positions(
        (
            lidar_display_x,
            lidar_display_y
        ),
        (
            lidar_display_x
            +
            heading_dx,
            lidar_display_y
            +
            heading_dy
        )
    )

    encoder_heading_arrow.set_positions(
        (
            encoder_display_x,
            encoder_display_y
        ),
        (
            encoder_display_x
            +
            heading_dx,
            encoder_display_y
            +
            heading_dy
        )
    )

    # Line between current estimates makes the difference easy
    # to see.
    difference_line.set_data(
        [
            lidar_display_x,
            encoder_display_x
        ],
        [
            lidar_display_y,
            encoder_display_y
        ]
    )

    update_route_limits(
        lidar_display_x,
        lidar_display_y,
        encoder_display_x,
        encoder_display_y
    )

    difference_distance = math.hypot(
        lidar_display_x
        -
        encoder_display_x,

        lidar_display_y
        -
        encoder_display_y
    )

    if len(difference_samples) > 0:

        mean_difference = float(
            np.mean(
                difference_samples
            )
        )

        max_difference = float(
            np.max(
                difference_samples
            )
        )

    else:

        mean_difference = 0.0
        max_difference = 0.0

    difference_label.config(
        text=(
            f"Current difference: "
            f"{difference_distance:.3f} m\n"
            f"Mean difference: "
            f"{mean_difference:.3f} m\n"
            f"Max difference: "
            f"{max_difference:.3f} m"
        )
    )

    lidar_pose_label.config(
        text=(
            f"LiDAR: "
            f"X={lidar_display_x:.3f}, "
            f"Y={lidar_display_y:.3f}"
        )
    )

    encoder_pose_label.config(
        text=(
            f"Encoder: "
            f"X={encoder_display_x:.3f}, "
            f"Y={encoder_display_y:.3f}"
        )
    )

    heading_label.config(
        text=(
            f"Shared navX Heading: "
            f"{current_heading:.2f}°"
        )
    )

    lidar_status_label.config(
        text=(
            f"LiDAR ICP: "
            f"{lidar_state}\n"
            f"Error: "
            f"{last_icp_error:.3f} m\n"
            f"Inliers: "
            f"{last_icp_inlier_ratio:.2f}"
        )
    )


# ============================================================
# LIVE LIDAR DISPLAY
# ============================================================

def update_live_lidar(
        local_points):

    if len(local_points) > 0:

        # Robot-relative view:
        #
        # forward -> screen right
        # right   -> screen down

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

    live_robot.set_data(
        [
            0.0
        ],
        [
            0.0
        ]
    )

    # This is robot-relative, therefore physical front always
    # points right in the LIVE LiDAR panel.
    live_forward_arrow.set_positions(
        (
            0.0,
            0.0
        ),
        (
            0.35,
            0.0
        )
    )


# ============================================================
# MAIN COMPARISON LOOP
# ============================================================

def comparison_update():

    global previous_heading

    if not program_running:
        return

    heading_before_update = (
        current_heading
    )

    heading_ok = (
        update_shared_heading()
    )

    if not heading_ok:

        overall_status_label.config(
            text="WAITING FOR navX"
        )

        root.after(
            TRACK_UPDATE_MS,
            comparison_update
        )

        return

    (
        angles,
        distances
    ) = get_full_scan()

    current_scan = scan_to_xy(
        angles,
        distances
    )

    update_live_lidar(
        current_scan
    )

    # --------------------------------------------------------
    # UPDATE BOTH METHODS FROM THE SAME ROBOT MOTION
    # --------------------------------------------------------

    update_lidar_localization(
        current_scan,
        heading_before_update
    )

    encoder_ok = update_encoder_localization(
        heading_before_update
    )

    # --------------------------------------------------------
    # LOG / ROUTE
    #
    # Q/E only:
    # both X/Y are locked, so no new translation route point
    # is necessary.
    #
    # During W+Q / W+E, translation_commanded() is true and
    # both methods continue tracking.
    # --------------------------------------------------------

    if translation_commanded():

        add_comparison_sample()

    update_comparison_artists()

    if encoder_ok:

        overall_status_label.config(
            text="COMPARISON ACTIVE"
        )

    else:

        overall_status_label.config(
            text="WAITING FOR ENCODERS"
        )

    previous_heading = (
        current_heading
    )

    canvas.draw_idle()

    root.after(
        TRACK_UPDATE_MS,
        comparison_update
    )


# ============================================================
# SAVE COMPARISON CSV
# ============================================================

def save_comparison_csv():

    if len(comparison_rows) == 0:

        save_status_label.config(
            text="Nothing to save yet."
        )

        return

    timestamp = time.strftime(
        "%Y%m%d_%H%M%S"
    )

    filename = (
        "lidar_vs_encoder_"
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

                "Lidar_X_m",
                "Lidar_Y_m",

                "Encoder_X_m",
                "Encoder_Y_m",

                "Heading_deg",

                "Difference_X_m",
                "Difference_Y_m",
                "Difference_Distance_m",

                "LidarValid",
                "LidarState",
                "ICP_Error_m",
                "ICP_InlierRatio",

                "Command_X",
                "Command_Y",
                "Command_Z"
            ]
        )

        for row in comparison_rows:

            writer.writerow(
                [
                    row[
                        "sample"
                    ],
                    row[
                        "elapsed"
                    ],

                    row[
                        "lidar_x"
                    ],
                    row[
                        "lidar_y"
                    ],

                    row[
                        "encoder_x"
                    ],
                    row[
                        "encoder_y"
                    ],

                    row[
                        "heading"
                    ],

                    row[
                        "difference_x"
                    ],
                    row[
                        "difference_y"
                    ],
                    row[
                        "difference_distance"
                    ],

                    row[
                        "lidar_valid"
                    ],
                    row[
                        "lidar_state"
                    ],
                    row[
                        "icp_error"
                    ],
                    row[
                        "icp_inlier_ratio"
                    ],

                    row[
                        "cmd_x"
                    ],
                    row[
                        "cmd_y"
                    ],
                    row[
                        "cmd_z"
                    ]
                ]
            )

    save_status_label.config(
        text=(
            "CSV saved:\n"
            +
            filename
        )
    )


# ============================================================
# SAVE COMPARISON IMAGE
# ============================================================

def save_comparison_image():

    timestamp = time.strftime(
        "%Y%m%d_%H%M%S"
    )

    filename = (
        "lidar_vs_encoder_"
        +
        timestamp
        +
        ".png"
    )

    path = os.path.join(
        SCRIPT_FOLDER,
        filename
    )

    figure.savefig(
        path,
        dpi=150
    )

    save_status_label.config(
        text=(
            "Image saved:\n"
            +
            filename
        )
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

    if key == "c":

        reset_comparison()

        return

    if key == "p":

        save_comparison_csv()

        return

    if key == "m":

        save_comparison_image()

        return

    if key == "space":

        space_stop_active = (
            True
        )

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

        space_stop_active = (
            False
        )

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

    program_running = (
        False
    )

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
    "Training Robot - LiDAR vs Encoder Localization Comparison"
)

root.geometry(
    "1600x850"
)

root.minsize(
    1350,
    720
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
    text="LiDAR vs ENCODER",
    font=(
        "Arial",
        17,
        "bold"
    )
)

title_label.pack(
    pady=7
)


controls_label = ttk.Label(
    control_frame,
    text=(
        "ONE KEYBOARD - BOTH METHODS\n\n"
        "W = Forward\n"
        "S = Backward\n"
        "A = Crab Left\n"
        "D = Crab Right\n"
        "Q = Rotate Left\n"
        "E = Rotate Right\n\n"
        "W+Q / W+E = Move + Turn\n\n"
        "Q/E only = X/Y LOCKED\n"
        "No keys = X/Y LOCKED\n\n"
        "SPACE = Stop immediately\n"
        "C = Reset BOTH to 0,0,0\n"
        "P = Save comparison CSV\n"
        "M = Save comparison image\n"
        "ESC = Close"
    ),
    justify="left"
)

controls_label.pack(
    pady=6
)


ttk.Separator(
    control_frame,
    orient="horizontal"
).pack(
    fill="x",
    pady=7
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
    pady=4
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
    pady=7
)


overall_status_label = ttk.Label(
    control_frame,
    text="WAITING",
    font=(
        "Arial",
        11,
        "bold"
    )
)

overall_status_label.pack(
    pady=4
)


heading_label = ttk.Label(
    control_frame,
    text="Shared navX Heading: 0.00°"
)

heading_label.pack(
    pady=2
)


lidar_pose_label = ttk.Label(
    control_frame,
    text="LiDAR: X=0.000, Y=0.000"
)

lidar_pose_label.pack()


encoder_pose_label = ttk.Label(
    control_frame,
    text="Encoder: X=0.000, Y=0.000"
)

encoder_pose_label.pack()


difference_label = ttk.Label(
    control_frame,
    text=(
        "Current difference: 0.000 m\n"
        "Mean difference: 0.000 m\n"
        "Max difference: 0.000 m"
    ),
    justify="center",
    font=(
        "Arial",
        10,
        "bold"
    )
)

difference_label.pack(
    pady=6
)


lidar_status_label = ttk.Label(
    control_frame,
    text=(
        "LiDAR ICP: WAITING\n"
        "Error: 999.000 m\n"
        "Inliers: 0.00"
    ),
    justify="center"
)

lidar_status_label.pack(
    pady=4
)


reset_status_label = ttk.Label(
    control_frame,
    text="Press C before test.",
    justify="center",
    wraplength=250
)

reset_status_label.pack(
    pady=4
)


save_status_label = ttk.Label(
    control_frame,
    text="No file saved.",
    justify="center",
    wraplength=250
)

save_status_label.pack(
    pady=4
)


# ============================================================
# SMALL LOG TABLE
# ============================================================

columns = (
    "sample",
    "lidar_x",
    "lidar_y",
    "encoder_x",
    "encoder_y",
    "difference"
)

log_table = ttk.Treeview(
    control_frame,
    columns=columns,
    show="headings",
    height=9
)

log_table.heading(
    "sample",
    text="#"
)

log_table.heading(
    "lidar_x",
    text="LX"
)

log_table.heading(
    "lidar_y",
    text="LY"
)

log_table.heading(
    "encoder_x",
    text="EX"
)

log_table.heading(
    "encoder_y",
    text="EY"
)

log_table.heading(
    "difference",
    text="Δ"
)

for column in columns:

    if column == "sample":

        width = 32

    else:

        width = 52

    log_table.column(
        column,
        width=width,
        anchor="center"
    )

log_table.pack(
    fill="x",
    pady=5
)


# ============================================================
# PLOT FRAME
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
        12,
        7.5
    ),
    dpi=100
)


live_axis = figure.add_subplot(
    121
)

comparison_axis = figure.add_subplot(
    122
)


# ============================================================
# LIVE LiDAR
# ============================================================

live_axis.set_title(
    "LIVE LiDAR - Current Obstacles"
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
    -LIVE_LIDAR_HALF_RANGE_M,
    LIVE_LIDAR_HALF_RANGE_M
)

live_axis.set_ylim(
    -LIVE_LIDAR_HALF_RANGE_M,
    LIVE_LIDAR_HALF_RANGE_M
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
    [
        0.0
    ],
    [
        0.0
    ],
    marker="o",
    linestyle="None",
    markersize=8
)


live_forward_arrow = FancyArrowPatch(
    (
        0.0,
        0.0
    ),
    (
        0.35,
        0.0
    ),
    arrowstyle="-|>",
    mutation_scale=16,
    linewidth=2
)

live_axis.add_patch(
    live_forward_arrow
)


# ============================================================
# COMPARISON ROUTE
# ============================================================

comparison_axis.set_title(
    "Same Drive: LiDAR vs Encoder Localization"
)

comparison_axis.set_xlabel(
    "Display X (m)"
)

comparison_axis.set_ylabel(
    "Display Y (m)"
)

comparison_axis.set_aspect(
    "equal",
    adjustable="box"
)

comparison_axis.set_xlim(
    route_min_x,
    route_max_x
)

comparison_axis.set_ylim(
    route_min_y,
    route_max_y
)

comparison_axis.grid(
    True
)


lidar_route_line, = comparison_axis.plot(
    lidar_route_x,
    lidar_route_y,
    linewidth=2,
    label="LiDAR ICP"
)


encoder_route_line, = comparison_axis.plot(
    encoder_route_x,
    encoder_route_y,
    linewidth=2,
    label="Encoder odometry"
)


difference_line, = comparison_axis.plot(
    [],
    [],
    linestyle="--",
    linewidth=1,
    label="Current difference"
)


lidar_marker, = comparison_axis.plot(
    [
        0.0
    ],
    [
        0.0
    ],
    marker="o",
    linestyle="None",
    markersize=8
)


encoder_marker, = comparison_axis.plot(
    [
        0.0
    ],
    [
        0.0
    ],
    marker="s",
    linestyle="None",
    markersize=8
)


lidar_heading_arrow = FancyArrowPatch(
    (
        0.0,
        0.0
    ),
    (
        HEADING_ARROW_LENGTH_M,
        0.0
    ),
    arrowstyle="-|>",
    mutation_scale=15,
    linewidth=2
)

comparison_axis.add_patch(
    lidar_heading_arrow
)


encoder_heading_arrow = FancyArrowPatch(
    (
        0.0,
        0.0
    ),
    (
        HEADING_ARROW_LENGTH_M,
        0.0
    ),
    arrowstyle="-|>",
    mutation_scale=15,
    linewidth=2
)

comparison_axis.add_patch(
    encoder_heading_arrow
)


comparison_axis.legend(
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


# ============================================================
# START LOOPS
# ============================================================

root.after(
    DRIVE_UPDATE_MS,
    update_drive
)

root.after(
    TRACK_UPDATE_MS,
    comparison_update
)


# ============================================================
# RUN
# ============================================================

try:

    root.mainloop()

finally:

    keyboard_table.putBoolean(
        "Enabled",
        False
    )

    send_stop()
