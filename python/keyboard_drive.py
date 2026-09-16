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


# ============================================================
# ROBOT CONNECTION
# ============================================================

ROBOT_IP = "10.23.45.2"


NetworkTables.initialize(
    server=ROBOT_IP
)


# ============================================================
# NETWORKTABLES
# ============================================================

keyboard_table = NetworkTables.getTable(
    "KeyboardDrive"
)


localization_table = NetworkTables.getTable(
    "LidarLocalization"
)




# ============================================================
# SAVE FOLDER
#
# Logs are saved directly inside the Python folder.
#
# Example:
#
# python/
# ├── keyboard_drive.py
# ├── lidar_localization_map.py
# └── localization_drive_log_20260914_123000.csv
# ============================================================

SCRIPT_FOLDER = os.path.dirname(
    os.path.abspath(__file__)
)


# ============================================================
# DRIVE SPEED
# ============================================================

MOVE_SPEED = 0.25

TURN_SPEED = 0.15

# Movement-state thresholds.
#
# Pure Q/E:
#     rotate in place
#     X/Y route should stay fixed
#
# W+Q, W+E, S+Q, S+E, A/D + Q/E:
#     translation + rotation together
#     route should form a curve
COMMAND_MOVE_THRESHOLD = 0.05
COMMAND_TURN_THRESHOLD = 0.05


# ============================================================
# UPDATE RATES
# ============================================================

# Keyboard -> robot:
# 20 Hz
DRIVE_UPDATE_MS = 50


# Localization logging:
#
# 5 Hz = one coordinate every 0.20 seconds
LOG_RATE_HZ = 5.0

LOG_UPDATE_MS = int(
    1000.0
    /
    LOG_RATE_HZ
)


# ============================================================
# TABLE DISPLAY
#
# Only latest rows are shown on screen.
#
# ALL rows are still kept in memory and saved to CSV.
# ============================================================

MAX_VISIBLE_LOG_ROWS = 15


# ============================================================
# ROUTE MAP SETTINGS
# ============================================================

INITIAL_MAP_HALF_RANGE_M = 1.5

MAP_EXPAND_MARGIN_M = 0.50

HEADING_ARROW_LENGTH_M = 0.25


# ============================================================
# KEY STATE
# ============================================================

pressed_keys = set()


space_stop_active = False


program_running = True


# ============================================================
# NETWORKTABLE HEARTBEAT
# ============================================================

heartbeat = 0


# ============================================================
# CURRENT DRIVE COMMAND
# ============================================================

current_command_x = 0.0

current_command_y = 0.0

current_command_z = 0.0


# ============================================================
# CURRENT LOCALIZATION
# ============================================================

current_robot_x = 0.0

current_robot_y = 0.0

current_robot_heading = 0.0

current_localization_valid = False


# ============================================================
# LIDAR LOCALIZATION ROUTE TRACKING
#
# IMPORTANT:
#
# X/Y come ONLY from LidarLocalization/RobotX and RobotY.
# These are LiDAR ICP translation values.
#
# Heading comes from LidarLocalization/RobotHeading, which is
# the navX heading used by lidar_localization_map.py.
#
# NO ENCODER X/Y IS USED HERE.
# ============================================================

route_pose_zero_initialized = False

route_pose_zero_x = 0.0

route_pose_zero_y = 0.0

route_pose_zero_heading = 0.0

raw_lidar_pose_x = 0.0

raw_lidar_pose_y = 0.0

raw_lidar_pose_heading = 0.0


# ============================================================
# LOGGING STORAGE
# ============================================================

log_rows = []

route_x = []

route_y = []


sample_counter = 0


test_start_time = time.monotonic()


# ============================================================
# MAP RANGE
# ============================================================

map_min_x = -INITIAL_MAP_HALF_RANGE_M

map_max_x = INITIAL_MAP_HALF_RANGE_M

map_min_y = -INITIAL_MAP_HALF_RANGE_M

map_max_y = INITIAL_MAP_HALF_RANGE_M


# ============================================================
# CALCULATE DRIVE COMMAND
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


    # ========================================================
    # W = FORWARD
    # ========================================================

    if "w" in pressed_keys:

        y += MOVE_SPEED


    # ========================================================
    # S = BACKWARD
    # ========================================================

    if "s" in pressed_keys:

        y -= MOVE_SPEED


    # ========================================================
    # A = CRAB LEFT
    # ========================================================

    if "a" in pressed_keys:

        x -= MOVE_SPEED


    # ========================================================
    # D = CRAB RIGHT
    # ========================================================

    if "d" in pressed_keys:

        x += MOVE_SPEED


    # ========================================================
    # Q = ROTATE LEFT
    # ========================================================

    if "q" in pressed_keys:

        z -= TURN_SPEED


    # ========================================================
    # E = ROTATE RIGHT
    # ========================================================

    if "e" in pressed_keys:

        z += TURN_SPEED


    return (
        x,
        y,
        z
    )


# ============================================================
# COMMAND STATE HELPERS
#
# These helpers make the intended behavior explicit:
#
# W/S/A/D:
#     translation only
#
# Q/E:
#     rotation only
#
# W+Q / W+E / S+Q / S+E / A/D+Q/E:
#     translation + rotation together
#     -> CURVED MOTION
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


def combined_move_and_turn_commanded():

    return (
        translation_commanded()
        and
        rotation_commanded()
    )


def robot_is_moving():

    return (
        translation_commanded()
        or
        rotation_commanded()
    )


def current_motion_text():

    if combined_move_and_turn_commanded():

        if current_command_y > COMMAND_MOVE_THRESHOLD:

            if current_command_z < -COMMAND_TURN_THRESHOLD:
                return "FORWARD + LEFT TURN"

            if current_command_z > COMMAND_TURN_THRESHOLD:
                return "FORWARD + RIGHT TURN"

        if current_command_y < -COMMAND_MOVE_THRESHOLD:

            if current_command_z < -COMMAND_TURN_THRESHOLD:
                return "BACKWARD + LEFT TURN"

            if current_command_z > COMMAND_TURN_THRESHOLD:
                return "BACKWARD + RIGHT TURN"

        return "MOVE + TURN"

    if pure_rotation_commanded():

        if current_command_z < 0.0:
            return "ROTATE LEFT IN PLACE"

        return "ROTATE RIGHT IN PLACE"

    if translation_commanded():

        if current_command_y > COMMAND_MOVE_THRESHOLD:
            return "FORWARD"

        if current_command_y < -COMMAND_MOVE_THRESHOLD:
            return "BACKWARD"

        if current_command_x < -COMMAND_MOVE_THRESHOLD:
            return "CRAB LEFT"

        if current_command_x > COMMAND_MOVE_THRESHOLD:
            return "CRAB RIGHT"

        return "TRANSLATING"

    return "IDLE"


# ============================================================
# WINDOW FOCUS
#
# Robot can only move while this application has focus.
#
# If user Alt+Tabs away:
#
# X = 0
# Y = 0
# Z = 0
# ============================================================

def keyboard_window_has_focus():

    try:

        return (
            root.focus_displayof()
            is not None
        )


    except tk.TclError:

        return False


# ============================================================
# SEND STOP
# ============================================================

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


    x_value_label.config(
        text="X: 0.00"
    )


    y_value_label.config(
        text="Y: 0.00"
    )


    z_value_label.config(
        text="Z: 0.00"
    )


    motion_value_label.config(
        text="Motion: IDLE"
    )


# ============================================================
# DRIVE UPDATE
#
# Runs at 20 Hz.
# ============================================================

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


    has_focus = (
        keyboard_window_has_focus()
    )


    # ========================================================
    # WINDOW NOT ACTIVE
    # ========================================================

    if not has_focus:

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


    # ========================================================
    # KEYBOARD ENABLED
    # ========================================================

    keyboard_table.putBoolean(
        "Enabled",
        True
    )


    (
        current_command_x,
        current_command_y,
        current_command_z
    ) = calculate_drive()


    # ========================================================
    # SEND COMMAND
    # ========================================================

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


    # ========================================================
    # DISPLAY COMMAND
    # ========================================================

    x_value_label.config(
        text=(
            f"X: "
            f"{current_command_x:.2f}"
        )
    )


    y_value_label.config(
        text=(
            f"Y: "
            f"{current_command_y:.2f}"
        )
    )


    z_value_label.config(
        text=(
            f"Z: "
            f"{current_command_z:.2f}"
        )
    )


    motion_value_label.config(
        text=(
            "Motion: "
            +
            current_motion_text()
        )
    )


    # ========================================================
    # DRIVE STATUS
    # ========================================================

    if space_stop_active:

        drive_status_label.config(
            text="STOPPED - SPACE"
        )


    elif combined_move_and_turn_commanded():

        drive_status_label.config(
            text=(
                "CURVING - "
                +
                current_motion_text()
            )
        )


    elif pure_rotation_commanded():

        drive_status_label.config(
            text=current_motion_text()
        )


    elif translation_commanded():

        drive_status_label.config(
            text=(
                "DRIVING - "
                +
                current_motion_text()
            )
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
# READ LIDAR LOCALIZATION
# ============================================================

def normalize_heading(
        angle_deg):

    while angle_deg > 180.0:

        angle_deg -= 360.0


    while angle_deg < -180.0:

        angle_deg += 360.0


    return angle_deg


def read_localization():

    global current_robot_x
    global current_robot_y
    global current_robot_heading
    global current_localization_valid

    global route_pose_zero_initialized
    global route_pose_zero_x
    global route_pose_zero_y
    global route_pose_zero_heading

    global raw_lidar_pose_x
    global raw_lidar_pose_y
    global raw_lidar_pose_heading


    # ========================================================
    # READ LIDAR ICP X/Y + NAVX HEADING
    # ========================================================

    raw_lidar_pose_x = localization_table.getNumber(
        "RobotX",
        raw_lidar_pose_x
    )

    raw_lidar_pose_y = localization_table.getNumber(
        "RobotY",
        raw_lidar_pose_y
    )

    raw_lidar_pose_heading = localization_table.getNumber(
        "RobotHeading",
        raw_lidar_pose_heading
    )

    current_localization_valid = localization_table.getBoolean(
        "LocalizationValid",
        False
    )


    # ========================================================
    # FIRST VALID READING = ROUTE ORIGIN
    # ========================================================

    if not route_pose_zero_initialized:

        route_pose_zero_x = float(
            raw_lidar_pose_x
        )

        route_pose_zero_y = float(
            raw_lidar_pose_y
        )

        route_pose_zero_heading = float(
            raw_lidar_pose_heading
        )

        route_pose_zero_initialized = True


    # ========================================================
    # RELATIVE WORLD POSITION
    # ========================================================

    delta_world_x = (
        raw_lidar_pose_x
        -
        route_pose_zero_x
    )

    delta_world_y = (
        raw_lidar_pose_y
        -
        route_pose_zero_y
    )


    # ========================================================
    # ROUTE STARTING HEADING FRAME
    #
    # This is a CONSTANT transform based on the heading when
    # the route was cleared. It does not continuously rotate
    # X/Y and therefore does not create tracking instability.
    # ========================================================

    zero_heading_rad = math.radians(
        route_pose_zero_heading
    )

    cos_zero = math.cos(
        zero_heading_rad
    )

    sin_zero = math.sin(
        zero_heading_rad
    )

    relative_robot_x = (
        delta_world_x
        *
        cos_zero
        -
        delta_world_y
        *
        sin_zero
    )

    relative_robot_y = (
        delta_world_x
        *
        sin_zero
        +
        delta_world_y
        *
        cos_zero
    )


    # ========================================================
    # DISPLAY AXIS
    #
    # Internal localization:
    # +X = robot right
    # +Y = robot forward
    #
    # Display:
    # 0° / forward = +X / right
    # 90°          = -Y / down
    # ========================================================

    current_robot_x = (
        relative_robot_y
    )

    current_robot_y = (
        -relative_robot_x
    )

    current_robot_heading = normalize_heading(
        raw_lidar_pose_heading
        -
        route_pose_zero_heading
    )


# ============================================================
# ADD ONE LOG SAMPLE
#
# Called only while robot is commanded to move.
# ============================================================

def add_log_sample():

    global sample_counter


    sample_counter += 1


    elapsed_time = (
        time.monotonic()
        -
        test_start_time
    )


    timestamp = time.strftime(
        "%H:%M:%S"
    )


    row = {
        "sample": sample_counter,
        "timestamp": timestamp,
        "elapsed": elapsed_time,

        "x": current_robot_x,
        "y": current_robot_y,
        "heading": current_robot_heading,

        "cmd_x": current_command_x,
        "cmd_y": current_command_y,
        "cmd_z": current_command_z,

        "motion": current_motion_text(),

        "valid": current_localization_valid
    }


    log_rows.append(
        row
    )


    # ========================================================
    # ROUTE
    #
    # We keep the coordinate even if LocalizationValid
    # temporarily becomes false.
    #
    # This is useful for your later analysis because the CSV
    # tells you which samples were VALID and which were HOLD.
    # ========================================================

    # --------------------------------------------------------
    # ROUTE GEOMETRY
    #
    # Pure Q/E:
    #   heading changes, but no new XY route point is added.
    #
    # W+Q / W+E etc:
    #   translation is present, so new points are added and the
    #   route naturally forms a curve from the localization pose.
    # --------------------------------------------------------

    if translation_commanded():

        route_x.append(
            current_robot_x
        )


        route_y.append(
            current_robot_y
        )


    add_row_to_table(
        row
    )


# ============================================================
# ADD ROW TO SMALL TABLE
# ============================================================

def add_row_to_table(
        row):

    valid_text = (
        "YES"
        if row["valid"]
        else
        "NO"
    )


    log_table.insert(
        "",
        "end",
        values=(
            row["sample"],
            f'{row["elapsed"]:.1f}',
            f'{row["x"]:.2f}',
            f'{row["y"]:.2f}',
            f'{row["heading"]:.1f}',
            valid_text
        )
    )


    # ========================================================
    # KEEP ONLY LATEST ROWS VISIBLE
    # ========================================================

    children = log_table.get_children()


    while (
        len(children)
        >
        MAX_VISIBLE_LOG_ROWS
    ):

        log_table.delete(
            children[0]
        )


        children = (
            log_table.get_children()
        )


    # Scroll automatically to newest row.
    children = log_table.get_children()


    if len(children) > 0:

        log_table.see(
            children[-1]
        )


# ============================================================
# UPDATE ROUTE MAP LIMITS
#
# We only expand the map.
#
# We do NOT shrink it continuously because that makes
# the display jump/shake.
# ============================================================

def update_map_limits():

    global map_min_x

    global map_max_x

    global map_min_y

    global map_max_y


    changed = False


    if (
        current_robot_x
        <
        map_min_x
        +
        MAP_EXPAND_MARGIN_M
    ):

        map_min_x = (
            current_robot_x
            -
            MAP_EXPAND_MARGIN_M
        )


        changed = True


    if (
        current_robot_x
        >
        map_max_x
        -
        MAP_EXPAND_MARGIN_M
    ):

        map_max_x = (
            current_robot_x
            +
            MAP_EXPAND_MARGIN_M
        )


        changed = True


    if (
        current_robot_y
        <
        map_min_y
        +
        MAP_EXPAND_MARGIN_M
    ):

        map_min_y = (
            current_robot_y
            -
            MAP_EXPAND_MARGIN_M
        )


        changed = True


    if (
        current_robot_y
        >
        map_max_y
        -
        MAP_EXPAND_MARGIN_M
    ):

        map_max_y = (
            current_robot_y
            +
            MAP_EXPAND_MARGIN_M
        )


        changed = True


    if changed:

        route_axis.set_xlim(
            map_min_x,
            map_max_x
        )


        route_axis.set_ylim(
            map_min_y,
            map_max_y
        )


# ============================================================
# UPDATE ROUTE DISPLAY
# ============================================================

def update_route_display():

    # ========================================================
    # ROUTE LINE
    # ========================================================

    route_line.set_data(
        route_x,
        route_y
    )


    # ========================================================
    # CURRENT ROBOT POSITION
    # ========================================================

    robot_marker.set_data(
        [current_robot_x],
        [current_robot_y]
    )


    # ========================================================
    # HEADING ARROW
    #
    # Display convention:
    #
    # 0°   = +X = RIGHT
    # 90°  = -Y = DOWN
    # 180° = -X = LEFT
    # 270° = +Y = UP
    #
    # navX positive direction is clockwise.
    # ========================================================

    heading_rad = math.radians(
        current_robot_heading
    )


    heading_dx = (
        HEADING_ARROW_LENGTH_M
        *
        math.cos(
            heading_rad
        )
    )


    heading_dy = (
        -HEADING_ARROW_LENGTH_M
        *
        math.sin(
            heading_rad
        )
    )


    heading_arrow.set_positions(
        (
            current_robot_x,
            current_robot_y
        ),
        (
            current_robot_x
            +
            heading_dx,
            current_robot_y
            +
            heading_dy
        )
    )


    # ========================================================
    # POSE LABEL
    # ========================================================

    robot_pose_text.set_position(
        (
            current_robot_x,
            current_robot_y - 0.12
        )
    )


    robot_pose_text.set_text(
        (
            f"X {current_robot_x:.2f}\n"
            f"Y {current_robot_y:.2f}\n"
            f"{current_robot_heading:.1f}°"
        )
    )


    update_map_limits()


    route_canvas.draw_idle()


# ============================================================
# LOCALIZATION / LOGGING LOOP
#
# This is the important section:
#
# 5 times per second:
#
# 1. Read LiDAR position
# 2. Update route map
# 3. If moving -> log coordinate
# ============================================================

def localization_update():

    if not program_running:

        return


    read_localization()


    # ========================================================
    # DISPLAY CURRENT POSE
    # ========================================================

    pose_x_label.config(
        text=(
            f"X: "
            f"{current_robot_x:.3f} m"
        )
    )


    pose_y_label.config(
        text=(
            f"Y: "
            f"{current_robot_y:.3f} m"
        )
    )


    pose_heading_label.config(
        text=(
            f"Heading: "
            f"{current_robot_heading:.2f}°"
        )
    )


    if current_localization_valid:

        localization_status_label.config(
            text="LIDAR LOCALIZATION: VALID"
        )


    else:

        localization_status_label.config(
            text="LIDAR LOCALIZATION: HOLD"
        )


    # ========================================================
    # LOG ONLY WHEN ROBOT IS MOVING
    # ========================================================

    if robot_is_moving():

        add_log_sample()


    # ========================================================
    # ROUTE IS ALWAYS UPDATED
    #
    # Robot marker still moves even when stopped.
    # ========================================================

    update_route_display()


    sample_count_label.config(
        text=(
            f"Logged samples: "
            f"{len(log_rows)}"
        )
    )


    root.after(
        LOG_UPDATE_MS,
        localization_update
    )


# ============================================================
# SAVE LOG TO CSV
#
# Press P
# ============================================================

def save_log():

    if len(log_rows) == 0:

        save_status_label.config(
            text="Nothing to save yet."
        )


        return


    timestamp = time.strftime(
        "%Y%m%d_%H%M%S"
    )


    filename = (
        "localization_drive_log_"
        +
        timestamp
        +
        ".csv"
    )


    file_path = os.path.join(
        SCRIPT_FOLDER,
        filename
    )


    with open(
        file_path,
        "w",
        newline=""
    ) as file:

        writer = csv.writer(
            file
        )


        writer.writerow(
            [
                "Sample",
                "Timestamp",
                "Elapsed_s",

                "X_m",
                "Y_m",
                "Heading_deg",

                "Command_X",
                "Command_Y",
                "Command_Z",

                "Motion",

                "LocalizationValid"
            ]
        )


        for row in log_rows:

            writer.writerow(
                [
                    row["sample"],
                    row["timestamp"],
                    row["elapsed"],

                    row["x"],
                    row["y"],
                    row["heading"],

                    row["cmd_x"],
                    row["cmd_y"],
                    row["cmd_z"],

                    row["motion"],

                    row["valid"]
                ]
            )


    save_status_label.config(
        text=(
            "SAVED:\n"
            +
            filename
        )
    )


    print(
        "=========================================="
    )


    print(
        "LOCALIZATION LOG SAVED"
    )


    print(
        file_path
    )


    print(
        "Samples:",
        len(log_rows)
    )


    print(
        "=========================================="
    )


# ============================================================
# CLEAR CURRENT TEST
#
# Press C
#
# This DOES NOT reset the LiDAR localization origin.
#
# It only clears:
#
# - route shown here
# - table
# - logging memory
#
# So you can start another recording without resetting SLAM.
# ============================================================

def clear_log():

    global log_rows
    global route_x
    global route_y
    global sample_counter
    global test_start_time

    global map_min_x
    global map_max_x
    global map_min_y
    global map_max_y

    global route_pose_zero_initialized
    global route_pose_zero_x
    global route_pose_zero_y
    global route_pose_zero_heading

    global raw_lidar_pose_x
    global raw_lidar_pose_y
    global raw_lidar_pose_heading

    global current_robot_x
    global current_robot_y
    global current_robot_heading


    # ========================================================
    # READ THE NEWEST LIDAR LOCALIZATION POSE
    # ========================================================

    raw_lidar_pose_x = localization_table.getNumber(
        "RobotX",
        raw_lidar_pose_x
    )

    raw_lidar_pose_y = localization_table.getNumber(
        "RobotY",
        raw_lidar_pose_y
    )

    raw_lidar_pose_heading = localization_table.getNumber(
        "RobotHeading",
        raw_lidar_pose_heading
    )


    # ========================================================
    # CURRENT LIDAR POSE BECOMES NEW ROUTE 0,0,0
    #
    # This does NOT reset the actual localization/ICP map.
    # It only resets the keyboard route display reference.
    # ========================================================

    route_pose_zero_x = float(
        raw_lidar_pose_x
    )

    route_pose_zero_y = float(
        raw_lidar_pose_y
    )

    route_pose_zero_heading = float(
        raw_lidar_pose_heading
    )

    route_pose_zero_initialized = True

    current_robot_x = 0.0
    current_robot_y = 0.0
    current_robot_heading = 0.0


    # ========================================================
    # CLEAR LOG STORAGE
    # ========================================================

    log_rows = []
    route_x = []
    route_y = []

    sample_counter = 0

    test_start_time = time.monotonic()


    # ========================================================
    # CLEAR TABLE
    # ========================================================

    for row_id in log_table.get_children():

        log_table.delete(
            row_id
        )


    # ========================================================
    # RESET ROUTE DISPLAY AROUND 0,0
    # ========================================================

    map_min_x = -INITIAL_MAP_HALF_RANGE_M
    map_max_x = INITIAL_MAP_HALF_RANGE_M

    map_min_y = -INITIAL_MAP_HALF_RANGE_M
    map_max_y = INITIAL_MAP_HALF_RANGE_M

    route_axis.set_xlim(
        map_min_x,
        map_max_x
    )

    route_axis.set_ylim(
        map_min_y,
        map_max_y
    )

    route_line.set_data(
        [],
        []
    )

    robot_marker.set_data(
        [0.0],
        [0.0]
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

    robot_pose_text.set_position(
        (
            0.0,
            -0.12
        )
    )

    robot_pose_text.set_text(
        "X 0.00\nY 0.00\n0.0°"
    )

    pose_x_label.config(
        text="X: 0.000 m"
    )

    pose_y_label.config(
        text="Y: 0.000 m"
    )

    pose_heading_label.config(
        text="Heading: 0.00°"
    )

    sample_count_label.config(
        text="Logged samples: 0"
    )

    save_status_label.config(
        text=(
            "Route cleared.\n"
            "Current LiDAR ICP pose = new 0,0,0."
        )
    )

    route_canvas.draw_idle()


# ============================================================
# KEY PRESS
# ============================================================

def on_key_press(
        event):

    global space_stop_active


    key = event.keysym.lower()


    # ========================================================
    # ESC = CLOSE
    # ========================================================

    if key == "escape":

        close_program()

        return


    # ========================================================
    # P = SAVE LOG
    # ========================================================

    if key == "p":

        save_log()

        return


    # ========================================================
    # C = CLEAR LOG / ROUTE
    # ========================================================

    if key == "c":

        clear_log()

        return


    # ========================================================
    # SPACE = STOP IMMEDIATELY
    # ========================================================

    if key == "space":

        space_stop_active = True


        pressed_keys.clear()


        send_stop()


        drive_status_label.config(
            text="STOPPED - SPACE"
        )


        return


    # ========================================================
    # DON'T ACCEPT MOVEMENT WHILE SPACE HELD
    # ========================================================

    if space_stop_active:

        return


    # ========================================================
    # MOVEMENT KEYS
    # ========================================================

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


# ============================================================
# KEY RELEASE
# ============================================================

def on_key_release(
        event):

    global space_stop_active


    key = event.keysym.lower()


    # ========================================================
    # SPACE RELEASE
    # ========================================================

    if key == "space":

        space_stop_active = False


        pressed_keys.clear()


        send_stop()


        drive_status_label.config(
            text="READY"
        )


        return


    # ========================================================
    # RELEASE MOVEMENT KEY
    # ========================================================

    if key in pressed_keys:

        pressed_keys.remove(
            key
        )


# ============================================================
# WINDOW CLICK
# ============================================================

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
# CREATE WINDOW
# ============================================================

root = tk.Tk()


root.title(
    "Training Robot - Keyboard Drive + Localization Logger"
)


root.geometry(
    "1450x720"
)


root.minsize(
    1200,
    650
)


root.protocol(
    "WM_DELETE_WINDOW",
    close_program
)


# ============================================================
# ROOT GRID
# ============================================================

root.columnconfigure(
    0,
    weight=0
)


root.columnconfigure(
    1,
    weight=1
)


root.columnconfigure(
    2,
    weight=1
)


root.rowconfigure(
    0,
    weight=1
)


# ============================================================
# LEFT PANEL
#
# KEYBOARD CONTROL
# ============================================================

control_frame = ttk.Frame(
    root,
    padding=15
)


control_frame.grid(
    row=0,
    column=0,
    sticky="nsew"
)


title_label = ttk.Label(
    control_frame,
    text="KEYBOARD DRIVE",
    font=(
        "Arial",
        18,
        "bold"
    )
)


title_label.pack(
    pady=10
)


controls_label = ttk.Label(
    control_frame,
    text=(
        "W = Forward\n"
        "S = Backward\n\n"
        "A = Crab Left\n"
        "D = Crab Right\n\n"
        "Q = Rotate Left\n"
        "E = Rotate Right\n\n"
        "W+Q / W+E = Forward Curve\n"
        "S+Q / S+E = Backward Curve\n"
        "A/D + Q/E = Crab + Turn\n\n"
        "Q/E only = Rotate in place\n\n"
        "SPACE = Stop Immediately\n\n"
        "P = Save Log\n"
        "C = Clear Route / Log\n"
        "ESC = Close"
    ),
    font=(
        "Arial",
        11
    ),
    justify="left"
)


controls_label.pack(
    pady=10
)


ttk.Separator(
    control_frame,
    orient="horizontal"
).pack(
    fill="x",
    pady=10
)


# ============================================================
# DRIVE STATUS
# ============================================================

drive_status_label = ttk.Label(
    control_frame,
    text="CLICK WINDOW TO DRIVE",
    font=(
        "Arial",
        12,
        "bold"
    )
)


drive_status_label.pack(
    pady=8
)


# ============================================================
# DRIVE VALUES
# ============================================================

x_value_label = ttk.Label(
    control_frame,
    text="X: 0.00"
)


x_value_label.pack()


y_value_label = ttk.Label(
    control_frame,
    text="Y: 0.00"
)


y_value_label.pack()


z_value_label = ttk.Label(
    control_frame,
    text="Z: 0.00"
)


z_value_label.pack()


motion_value_label = ttk.Label(
    control_frame,
    text="Motion: IDLE",
    font=(
        "Arial",
        10,
        "bold"
    )
)

motion_value_label.pack(
    pady=4
)


ttk.Separator(
    control_frame,
    orient="horizontal"
).pack(
    fill="x",
    pady=10
)


# ============================================================
# LOCALIZATION INFORMATION
# ============================================================

localization_title = ttk.Label(
    control_frame,
    text="LiDAR ICP + navX Tracking",
    font=(
        "Arial",
        11,
        "bold"
    )
)


localization_title.pack(
    pady=5
)


pose_x_label = ttk.Label(
    control_frame,
    text="X: 0.000 m"
)


pose_x_label.pack()


pose_y_label = ttk.Label(
    control_frame,
    text="Y: 0.000 m"
)


pose_y_label.pack()


pose_heading_label = ttk.Label(
    control_frame,
    text="Heading: 0.00°"
)


pose_heading_label.pack()


localization_status_label = ttk.Label(
    control_frame,
    text="LIDAR LOCALIZATION: HOLD",
    font=(
        "Arial",
        10,
        "bold"
    )
)


localization_status_label.pack(
    pady=5
)


sample_count_label = ttk.Label(
    control_frame,
    text="Logged samples: 0"
)


sample_count_label.pack(
    pady=4
)


logging_rate_label = ttk.Label(
    control_frame,
    text=(
        f"Logging rate: "
        f"{LOG_RATE_HZ:.1f} Hz"
    )
)


logging_rate_label.pack()


save_status_label = ttk.Label(
    control_frame,
    text="No log saved yet.",
    wraplength=220,
    justify="center"
)


save_status_label.pack(
    pady=12
)


# ============================================================
# CENTRE PANEL
#
# ROUTE MAP
# ============================================================

map_frame = ttk.Frame(
    root,
    padding=8
)


map_frame.grid(
    row=0,
    column=1,
    sticky="nsew"
)


map_frame.columnconfigure(
    0,
    weight=1
)


map_frame.rowconfigure(
    1,
    weight=1
)


map_title = ttk.Label(
    map_frame,
    text="LIVE LOCALIZATION ROUTE",
    font=(
        "Arial",
        14,
        "bold"
    )
)


map_title.grid(
    row=0,
    column=0,
    pady=5
)


# ============================================================
# MATPLOTLIB FIGURE
# ============================================================

route_figure = Figure(
    figsize=(6, 6),
    dpi=100
)


route_axis = route_figure.add_subplot(
    111
)


route_axis.set_title(
    "Robot X-Y Track"
)


route_axis.set_xlabel(
    "Display X (m)"
)


route_axis.set_ylabel(
    "Display Y (m)"
)


route_axis.set_aspect(
    "equal",
    adjustable="box"
)


route_axis.grid(
    True
)


route_axis.set_xlim(
    map_min_x,
    map_max_x
)


route_axis.set_ylim(
    map_min_y,
    map_max_y
)


# ============================================================
# ROUTE ARTISTS
# ============================================================

route_line, = route_axis.plot(
    [],
    [],
    linewidth=2,
    label="Robot route"
)


robot_marker, = route_axis.plot(
    [],
    [],
    marker="o",
    linestyle="None",
    markersize=8,
    label="Robot"
)


heading_arrow = FancyArrowPatch(
    (0.0, 0.0),
    (HEADING_ARROW_LENGTH_M, 0.0),
    arrowstyle="-|>",
    mutation_scale=16,
    linewidth=2
)


route_axis.add_patch(
    heading_arrow
)


robot_pose_text = route_axis.text(
    0.0,
    0.0,
    "",
    fontsize=8,
    ha="center",
    va="top",
    bbox=dict(
        boxstyle="round,pad=0.2",
        facecolor="white",
        edgecolor="black",
        alpha=0.90
    )
)


route_axis.legend(
    loc="upper right"
)


route_canvas = FigureCanvasTkAgg(
    route_figure,
    master=map_frame
)


route_canvas.get_tk_widget().grid(
    row=1,
    column=0,
    sticky="nsew"
)


# ============================================================
# RIGHT PANEL
#
# COORDINATE LOG TABLE
# ============================================================

table_frame = ttk.Frame(
    root,
    padding=8
)


table_frame.grid(
    row=0,
    column=2,
    sticky="nsew"
)


table_frame.columnconfigure(
    0,
    weight=1
)


table_frame.rowconfigure(
    1,
    weight=1
)


table_title = ttk.Label(
    table_frame,
    text="COORDINATE LOG",
    font=(
        "Arial",
        14,
        "bold"
    )
)


table_title.grid(
    row=0,
    column=0,
    pady=5
)


# ============================================================
# TREEVIEW
# ============================================================

columns = (
    "sample",
    "time",
    "x",
    "y",
    "heading",
    "valid"
)


log_table = ttk.Treeview(
    table_frame,
    columns=columns,
    show="headings",
    height=22
)


log_table.heading(
    "sample",
    text="#"
)


log_table.heading(
    "time",
    text="Time"
)


log_table.heading(
    "x",
    text="X (m)"
)


log_table.heading(
    "y",
    text="Y (m)"
)


log_table.heading(
    "heading",
    text="Heading"
)


log_table.heading(
    "valid",
    text="Valid"
)


log_table.column(
    "sample",
    width=45,
    anchor="center"
)


log_table.column(
    "time",
    width=65,
    anchor="center"
)


log_table.column(
    "x",
    width=75,
    anchor="center"
)


log_table.column(
    "y",
    width=75,
    anchor="center"
)


log_table.column(
    "heading",
    width=80,
    anchor="center"
)


log_table.column(
    "valid",
    width=55,
    anchor="center"
)


# ============================================================
# TABLE SCROLLBAR
# ============================================================

table_scroll = ttk.Scrollbar(
    table_frame,
    orient="vertical",
    command=log_table.yview
)


log_table.configure(
    yscrollcommand=table_scroll.set
)


log_table.grid(
    row=1,
    column=0,
    sticky="nsew"
)


table_scroll.grid(
    row=1,
    column=1,
    sticky="ns"
)


table_info = ttk.Label(
    table_frame,
    text=(
        "W/A/S/D + Q/E = translation + rotation curve.\n"
        "Q/E only = heading changes without XY route movement."
    ),
    justify="center"
)


table_info.grid(
    row=2,
    column=0,
    pady=10
)


# ============================================================
# KEY EVENTS
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
    LOG_UPDATE_MS,
    localization_update
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