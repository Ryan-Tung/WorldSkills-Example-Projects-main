import os
import csv
import time
import math
import tkinter as tk
from tkinter import ttk

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

# Python -> Java KeyboardDrive command
keyboard_table = NetworkTables.getTable(
    "KeyboardDrive"
)

# Java DriveTrain -> Python
# Heading = navX corrected heading
robot_pose_table = NetworkTables.getTable(
    "RobotPose"
)

# Raw wheel encoder distances are already published by
# DriveTrain.java to the Training Robot Shuffleboard tab.
#
# We read the three wheel encoders directly here so this
# encoder-only localization is NOT affected by the older
# RobotPose Q/E XY-hold filter used by the normal keyboard
# tracking mode.
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
# DRIVE SETTINGS
# ============================================================

MOVE_SPEED = 0.25
TURN_SPEED = 0.15

DRIVE_UPDATE_MS = 50

LOG_RATE_HZ = 5.0
LOG_UPDATE_MS = int(
    1000.0 / LOG_RATE_HZ
)

COMMAND_THRESHOLD = 0.05

# Encoder-only odometry should follow the wheel measurements
# even during Q/E rotation and even when no keyboard key is
# being pressed.  Only extremely tiny per-update movement is
# treated as encoder noise.
ENCODER_DELTA_DEADBAND_M = 0.001

# Used only to decide when a heading-only update should be
# written into the coordinate log.  It does NOT freeze heading.
HEADING_LOG_DEADBAND_DEG = 0.50


# ============================================================
# ROUTE DISPLAY SETTINGS
# ============================================================

MAX_VISIBLE_LOG_ROWS = 15

INITIAL_MAP_HALF_RANGE_M = 1.5
MAP_EXPAND_MARGIN_M = 0.50
HEADING_ARROW_LENGTH_M = 0.25


# ============================================================
# KEY / PROGRAM STATE
# ============================================================

pressed_keys = set()
space_stop_active = False
program_running = True
heartbeat = 0

current_command_x = 0.0
current_command_y = 0.0
current_command_z = 0.0


# ============================================================
# RAW ENCODER / NAVX DATA FROM JAVA
# ============================================================

# DriveTrain.java publishes these encoder distances to the
# Training Robot Shuffleboard tab.  They are in the same
# distance unit used by DriveTrain odometry (millimetres in
# this project).
raw_left_encoder = 0.0
raw_right_encoder = 0.0
raw_back_encoder = 0.0
raw_robot_pose_heading = 0.0
encoder_data_available = False


# ============================================================
# DISPLAYED ENCODER LOCALIZATION POSE
#
# IMPORTANT:
#
# This is genuine encoder-only X/Y odometry:
#
# X/Y     = calculated directly from the 3 raw wheel encoders
# Heading = navX
#
# LiDAR / ICP are NOT used.
# RobotPose X/Y are NOT used.
# ============================================================

current_robot_x = 0.0
current_robot_y = 0.0
current_robot_heading = 0.0


# ============================================================
# ROUTE REFERENCE / PREVIOUS ENCODER STATE
#
# Unlike the older LiDAR/keyboard tracking fixes, we do NOT
# force X/Y to hold simply because Q/E is pressed or because
# no keyboard key is pressed.
#
# Every real encoder movement is allowed to affect X/Y.
# Only a very small movement deadband removes encoder noise.
# ============================================================

route_reference_initialized = False
route_zero_heading = 0.0

previous_left_encoder = 0.0
previous_right_encoder = 0.0
previous_back_encoder = 0.0
previous_relative_heading = 0.0

last_translation_accepted = False
last_translation_magnitude_m = 0.0
last_heading_delta_deg = 0.0


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
# DRIVE COMMAND CALCULATION
# ============================================================

def calculate_drive():

    if space_stop_active:
        return 0.0, 0.0, 0.0

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

    return x, y, z


# ============================================================
# COMMAND TYPE HELPERS
# ============================================================

def translation_commanded():

    return (
        abs(current_command_x) >= COMMAND_THRESHOLD
        or
        abs(current_command_y) >= COMMAND_THRESHOLD
    )


def rotation_commanded():

    return (
        abs(current_command_z) >= COMMAND_THRESHOLD
    )


def pure_rotation_commanded():

    return (
        rotation_commanded()
        and
        not translation_commanded()
    )


def robot_is_commanded_to_move():

    return (
        translation_commanded()
        or
        rotation_commanded()
    )


# ============================================================
# WINDOW FOCUS SAFETY
# ============================================================

def keyboard_window_has_focus():

    try:
        return root.focus_displayof() is not None
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

    keyboard_table.putNumber("X", 0.0)
    keyboard_table.putNumber("Y", 0.0)
    keyboard_table.putNumber("Z", 0.0)

    x_value_label.config(text="X: 0.00")
    y_value_label.config(text="Y: 0.00")
    z_value_label.config(text="Z: 0.00")


# ============================================================
# DRIVE UPDATE
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

    x_value_label.config(
        text=f"X: {current_command_x:.2f}"
    )

    y_value_label.config(
        text=f"Y: {current_command_y:.2f}"
    )

    z_value_label.config(
        text=f"Z: {current_command_z:.2f}"
    )

    if space_stop_active:

        drive_status_label.config(
            text="STOPPED - SPACE"
        )

    elif translation_commanded() and rotation_commanded():

        drive_status_label.config(
            text="DRIVING + TURNING"
        )

    elif translation_commanded():

        drive_status_label.config(
            text="DRIVING"
        )

    elif pure_rotation_commanded():

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
# ANGLE NORMALIZATION
# ============================================================

def normalize_heading(angle_deg):

    while angle_deg > 180.0:
        angle_deg -= 360.0

    while angle_deg < -180.0:
        angle_deg += 360.0

    return angle_deg


# ============================================================
# READ RAW ENCODERS + NAVX HEADING
# ============================================================

def read_raw_encoder_state():

    global raw_left_encoder
    global raw_right_encoder
    global raw_back_encoder
    global raw_robot_pose_heading
    global encoder_data_available

    raw_left = training_robot_table.getNumber(
        "Left Encoder",
        999999.0
    )

    raw_right = training_robot_table.getNumber(
        "Right Encoder",
        999999.0
    )

    raw_back = training_robot_table.getNumber(
        "Back Encoder",
        999999.0
    )

    raw_heading = robot_pose_table.getNumber(
        "Heading",
        999999.0
    )

    encoder_data_available = (
        raw_left != 999999.0
        and
        raw_right != 999999.0
        and
        raw_back != 999999.0
        and
        raw_heading != 999999.0
    )

    if not encoder_data_available:
        return False

    raw_left_encoder = float(raw_left)
    raw_right_encoder = float(raw_right)
    raw_back_encoder = float(raw_back)
    raw_robot_pose_heading = float(raw_heading)

    return True


# ============================================================
# INITIALIZE ROUTE REFERENCE
# ============================================================

def initialize_route_reference():

    global route_reference_initialized
    global route_zero_heading

    global previous_left_encoder
    global previous_right_encoder
    global previous_back_encoder
    global previous_relative_heading

    global current_robot_x
    global current_robot_y
    global current_robot_heading

    global last_translation_accepted
    global last_translation_magnitude_m
    global last_heading_delta_deg

    global route_x
    global route_y

    if not read_raw_encoder_state():
        return False

    route_zero_heading = raw_robot_pose_heading

    previous_left_encoder = raw_left_encoder
    previous_right_encoder = raw_right_encoder
    previous_back_encoder = raw_back_encoder

    previous_relative_heading = 0.0

    current_robot_x = 0.0
    current_robot_y = 0.0
    current_robot_heading = 0.0

    last_translation_accepted = False
    last_translation_magnitude_m = 0.0
    last_heading_delta_deg = 0.0

    if len(route_x) == 0:
        route_x.append(0.0)
        route_y.append(0.0)

    route_reference_initialized = True

    return True


# ============================================================
# UPDATE ENCODER-ONLY LOCALIZATION
#
# This performs the same 3-wheel odometry calculation used by
# DriveTrain.java, but directly from the raw wheel encoder
# distances.
#
# There is NO command-based XY hold here.
#
# Therefore:
#
# Q/E only:
#   - heading changes
#   - if the real robot shifts and the encoders measure it,
#     X/Y are allowed to change too
#
# No keyboard key:
#   - if encoders do not move, X/Y stay still
#   - tiny encoder noise is removed by the deadband
#   - if the wheels genuinely roll, odometry records it
#
# W+Q / W+E:
#   - translation and rotation are both tracked
#   - midpoint heading is used for better curved-path tracking
# ============================================================

def update_encoder_localization():

    global previous_left_encoder
    global previous_right_encoder
    global previous_back_encoder
    global previous_relative_heading

    global current_robot_x
    global current_robot_y
    global current_robot_heading

    global last_translation_accepted
    global last_translation_magnitude_m
    global last_heading_delta_deg

    if not read_raw_encoder_state():
        return False

    if not route_reference_initialized:

        if not initialize_route_reference():
            return False

        return True

    # --------------------------------------------------------
    # WHEEL MOVEMENT SINCE PREVIOUS PYTHON UPDATE
    # --------------------------------------------------------

    delta_left = (
        raw_left_encoder
        -
        previous_left_encoder
    )

    delta_right = (
        raw_right_encoder
        -
        previous_right_encoder
    )

    delta_back = (
        raw_back_encoder
        -
        previous_back_encoder
    )

    # Always consume the latest encoder readings.
    previous_left_encoder = raw_left_encoder
    previous_right_encoder = raw_right_encoder
    previous_back_encoder = raw_back_encoder

    # --------------------------------------------------------
    # CURRENT HEADING RELATIVE TO THE ROUTE RESET
    # --------------------------------------------------------

    new_relative_heading = normalize_heading(
        raw_robot_pose_heading
        -
        route_zero_heading
    )

    heading_delta = normalize_heading(
        new_relative_heading
        -
        previous_relative_heading
    )

    last_heading_delta_deg = heading_delta

    # Use the middle of the heading change while transforming
    # this encoder interval.  This is more accurate when the
    # robot is translating and rotating at the same time.
    midpoint_heading = normalize_heading(
        previous_relative_heading
        +
        (heading_delta * 0.5)
    )

    previous_relative_heading = new_relative_heading
    current_robot_heading = new_relative_heading

    # --------------------------------------------------------
    # 3-WHEEL HOLONOMIC ODOMETRY
    #
    # localY = forward/back movement
    # localX = sideways/crab movement
    #
    # Encoder distances are in millimetres in this project.
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

    local_x_m = local_x_mm / 1000.0
    local_y_m = local_y_mm / 1000.0

    translation_magnitude = math.hypot(
        local_x_m,
        local_y_m
    )

    last_translation_magnitude_m = translation_magnitude

    # --------------------------------------------------------
    # SMALL ENCODER-NOISE DEADBAND
    #
    # IMPORTANT:
    # We do NOT check W/A/S/D/Q/E here.
    # The encoder decides whether translation happened.
    # --------------------------------------------------------

    if translation_magnitude < ENCODER_DELTA_DEADBAND_M:

        last_translation_accepted = False

        return True

    last_translation_accepted = True

    # --------------------------------------------------------
    # LOCAL ROBOT MOVEMENT -> WORLD MOVEMENT
    # --------------------------------------------------------

    heading_rad = math.radians(
        midpoint_heading
    )

    world_delta_x = (
        local_x_m
        *
        math.cos(heading_rad)
        +
        local_y_m
        *
        math.sin(heading_rad)
    )

    world_delta_y = (
        -local_x_m
        *
        math.sin(heading_rad)
        +
        local_y_m
        *
        math.cos(heading_rad)
    )

    # --------------------------------------------------------
    # WORLD FRAME -> USER DISPLAY FRAME
    #
    #                  270°
    #                    ↑
    #                    |
    # 180°  <------------+------------> 0° / +X
    #                    |
    #                    ↓
    #                   90°
    #
    # Forward at route reset = +Display X.
    # --------------------------------------------------------

    display_delta_x = world_delta_y
    display_delta_y = -world_delta_x

    current_robot_x += display_delta_x
    current_robot_y += display_delta_y

    return True


# ============================================================
# ADD LOG SAMPLE
#
# Route points are added when the encoders measure meaningful
# translation, or when navX measures a meaningful heading change.
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
        "translation_accepted": last_translation_accepted,
        "translation_magnitude_m": last_translation_magnitude_m,
        "heading_delta_deg": last_heading_delta_deg
    }

    log_rows.append(row)

    route_x.append(
        current_robot_x
    )

    route_y.append(
        current_robot_y
    )

    add_row_to_table(row)


# ============================================================
# ADD ROW TO TABLE
# ============================================================

def add_row_to_table(row):

    moved = row["translation_accepted"]
    turned = (
        abs(row["heading_delta_deg"])
        >=
        HEADING_LOG_DEADBAND_DEG
    )

    if moved and turned:
        motion_text = "MOVE+TURN"
    elif moved:
        motion_text = "MOVE"
    elif turned:
        motion_text = "TURN"
    else:
        motion_text = "IDLE"

    log_table.insert(
        "",
        "end",
        values=(
            row["sample"],
            f'{row["elapsed"]:.1f}',
            f'{row["x"]:.3f}',
            f'{row["y"]:.3f}',
            f'{row["heading"]:.1f}',
            motion_text
        )
    )

    children = log_table.get_children()

    while len(children) > MAX_VISIBLE_LOG_ROWS:

        log_table.delete(
            children[0]
        )

        children = log_table.get_children()

    children = log_table.get_children()

    if len(children) > 0:
        log_table.see(children[-1])


# ============================================================
# MAP LIMITS
# ============================================================

def update_map_limits():

    global map_min_x
    global map_max_x
    global map_min_y
    global map_max_y

    changed = False

    if current_robot_x < map_min_x + MAP_EXPAND_MARGIN_M:
        map_min_x = current_robot_x - MAP_EXPAND_MARGIN_M
        changed = True

    if current_robot_x > map_max_x - MAP_EXPAND_MARGIN_M:
        map_max_x = current_robot_x + MAP_EXPAND_MARGIN_M
        changed = True

    if current_robot_y < map_min_y + MAP_EXPAND_MARGIN_M:
        map_min_y = current_robot_y - MAP_EXPAND_MARGIN_M
        changed = True

    if current_robot_y > map_max_y - MAP_EXPAND_MARGIN_M:
        map_max_y = current_robot_y + MAP_EXPAND_MARGIN_M
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
# ROUTE DISPLAY
# ============================================================

def update_route_display():

    route_line.set_data(
        route_x,
        route_y
    )

    robot_marker.set_data(
        [current_robot_x],
        [current_robot_y]
    )

    heading_rad = math.radians(
        current_robot_heading
    )

    heading_dx = (
        HEADING_ARROW_LENGTH_M
        *
        math.cos(heading_rad)
    )

    heading_dy = (
        -HEADING_ARROW_LENGTH_M
        *
        math.sin(heading_rad)
    )

    heading_arrow.set_positions(
        (
            current_robot_x,
            current_robot_y
        ),
        (
            current_robot_x + heading_dx,
            current_robot_y + heading_dy
        )
    )

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
# ENCODER LOCALIZATION LOOP
# ============================================================

def localization_update():

    if not program_running:
        return

    pose_ok = update_encoder_localization()

    if pose_ok:

        pose_status_label.config(
            text="ENCODER ODOMETRY: CONNECTED"
        )

        pose_x_label.config(
            text=f"X: {current_robot_x:.3f} m"
        )

        pose_y_label.config(
            text=f"Y: {current_robot_y:.3f} m"
        )

        pose_heading_label.config(
            text=f"Heading: {current_robot_heading:.2f}°"
        )

        # ----------------------------------------------------
        # LOG WHAT THE SENSORS ACTUALLY MEASURE
        #
        # - real encoder translation -> log it
        # - heading-only rotation -> log heading as TURN
        # - no command gating is used for X/Y localization
        # ----------------------------------------------------

        if (
            last_translation_accepted
            or
            abs(last_heading_delta_deg)
            >=
            HEADING_LOG_DEADBAND_DEG
        ):
            add_log_sample()

    else:

        pose_status_label.config(
            text="ENCODER ODOMETRY: NO ENCODER / NAVX DATA"
        )

    update_route_display()

    sample_count_label.config(
        text=f"Logged samples: {len(log_rows)}"
    )

    root.after(
        LOG_UPDATE_MS,
        localization_update
    )


# ============================================================
# SAVE CSV
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
        "encoder_localization_log_"
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

        writer = csv.writer(file)

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
                "TranslationAccepted",
                "TranslationMagnitude_m",
                "HeadingDelta_deg"
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
                    row["translation_accepted"],
                    row["translation_magnitude_m"],
                    row["heading_delta_deg"]
                ]
            )

    save_status_label.config(
        text=(
            "SAVED:\n"
            +
            filename
        )
    )

    print("==========================================")
    print("ENCODER LOCALIZATION LOG SAVED")
    print(file_path)
    print("Samples:", len(log_rows))
    print("==========================================")


# ============================================================
# CLEAR / RESET ROUTE
#
# C does NOT change the working Java drivetrain logic.
# It only creates a fresh Python encoder-localization origin.
#
# Current encoder readings become the new baseline and the
# current navX direction becomes heading 0°.
# ============================================================

def clear_route():

    global log_rows
    global route_x
    global route_y
    global sample_counter
    global test_start_time

    global map_min_x
    global map_max_x
    global map_min_y
    global map_max_y

    global route_reference_initialized
    global route_zero_heading

    global previous_left_encoder
    global previous_right_encoder
    global previous_back_encoder
    global previous_relative_heading

    global current_robot_x
    global current_robot_y
    global current_robot_heading

    global last_translation_accepted
    global last_translation_magnitude_m
    global last_heading_delta_deg

    if not read_raw_encoder_state():

        save_status_label.config(
            text="Cannot reset: encoder/navX data unavailable."
        )

        return

    route_zero_heading = raw_robot_pose_heading

    previous_left_encoder = raw_left_encoder
    previous_right_encoder = raw_right_encoder
    previous_back_encoder = raw_back_encoder
    previous_relative_heading = 0.0

    route_reference_initialized = True

    current_robot_x = 0.0
    current_robot_y = 0.0
    current_robot_heading = 0.0

    last_translation_accepted = False
    last_translation_magnitude_m = 0.0
    last_heading_delta_deg = 0.0

    log_rows = []

    # Keep visible route origin at 0,0.
    route_x = [0.0]
    route_y = [0.0]

    sample_counter = 0
    test_start_time = time.monotonic()

    for row_id in log_table.get_children():
        log_table.delete(row_id)

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
        route_x,
        route_y
    )

    robot_marker.set_data(
        [0.0],
        [0.0]
    )

    heading_arrow.set_positions(
        (0.0, 0.0),
        (HEADING_ARROW_LENGTH_M, 0.0)
    )

    robot_pose_text.set_position(
        (0.0, -0.12)
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
            "Route reset.\n"
            "Current wheel encoders + navX = new 0,0,0."
        )
    )

    route_canvas.draw_idle()


# ============================================================
# KEY PRESS
# ============================================================

def on_key_press(event):

    global space_stop_active

    key = event.keysym.lower()

    if key == "escape":
        close_program()
        return

    if key == "p":
        save_log()
        return

    if key == "c":
        clear_route()
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
        pressed_keys.add(key)


# ============================================================
# KEY RELEASE
# ============================================================

def on_key_release(event):

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
        pressed_keys.remove(key)


# ============================================================
# WINDOW CLICK
# ============================================================

def on_window_click(event):

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
# WINDOW
# ============================================================

root = tk.Tk()

root.title(
    "Training Robot - Encoder Localization + Keyboard Drive"
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

root.columnconfigure(0, weight=0)
root.columnconfigure(1, weight=1)
root.columnconfigure(2, weight=1)
root.rowconfigure(0, weight=1)


# ============================================================
# LEFT PANEL
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
    text="ENCODER LOCALIZATION",
    font=("Arial", 18, "bold")
)

title_label.pack(pady=10)


controls_label = ttk.Label(
    control_frame,
    text=(
        "W = Forward\n"
        "S = Backward\n\n"
        "A = Crab Left\n"
        "D = Crab Right\n\n"
        "Q = Rotate Left\n"
        "E = Rotate Right\n\n"
        "SPACE = Stop Immediately\n\n"
        "P = Save Log\n"
        "C = Reset Route to 0,0,0\n"
        "ESC = Close"
    ),
    font=("Arial", 11),
    justify="left"
)

controls_label.pack(pady=10)


ttk.Separator(
    control_frame,
    orient="horizontal"
).pack(
    fill="x",
    pady=10
)


drive_status_label = ttk.Label(
    control_frame,
    text="CLICK WINDOW TO DRIVE",
    font=("Arial", 12, "bold")
)

drive_status_label.pack(pady=8)


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


ttk.Separator(
    control_frame,
    orient="horizontal"
).pack(
    fill="x",
    pady=10
)


source_title = ttk.Label(
    control_frame,
    text="Localization Source",
    font=("Arial", 11, "bold")
)

source_title.pack(pady=5)


source_label = ttk.Label(
    control_frame,
    text=(
        "X/Y: 3 raw wheel encoders\n"
        "Heading: navX\n"
        "RobotPose X/Y: NOT USED\n"
        "LiDAR / ICP: NOT USED"
    ),
    justify="center"
)

source_label.pack(pady=5)


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


pose_status_label = ttk.Label(
    control_frame,
    text="ENCODER ODOMETRY: WAITING",
    font=("Arial", 10, "bold")
)

pose_status_label.pack(pady=5)


sample_count_label = ttk.Label(
    control_frame,
    text="Logged samples: 0"
)

sample_count_label.pack(pady=4)


logging_rate_label = ttk.Label(
    control_frame,
    text=f"Tracking rate: {LOG_RATE_HZ:.1f} Hz"
)

logging_rate_label.pack()


filter_label = ttk.Label(
    control_frame,
    text=(
        "Encoder behavior:\n"
        "Q/E → real encoder XY is kept\n"
        "Idle → only tiny encoder noise removed\n"
        f"XY deadband: {ENCODER_DELTA_DEADBAND_M*1000.0:.1f} mm"
    ),
    justify="center"
)

filter_label.pack(pady=8)


save_status_label = ttk.Label(
    control_frame,
    text="No log saved yet.",
    wraplength=240,
    justify="center"
)

save_status_label.pack(pady=10)


# ============================================================
# CENTRE PANEL - ROUTE
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

map_frame.columnconfigure(0, weight=1)
map_frame.rowconfigure(1, weight=1)


map_title = ttk.Label(
    map_frame,
    text="LIVE ENCODER ODOMETRY ROUTE",
    font=("Arial", 14, "bold")
)

map_title.grid(
    row=0,
    column=0,
    pady=5
)


route_figure = Figure(
    figsize=(6, 6),
    dpi=100
)

route_axis = route_figure.add_subplot(111)

route_axis.set_title(
    "Encoder X/Y + navX Heading"
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

route_axis.grid(True)

route_axis.set_xlim(
    map_min_x,
    map_max_x
)

route_axis.set_ylim(
    map_min_y,
    map_max_y
)


route_line, = route_axis.plot(
    route_x,
    route_y,
    linewidth=2,
    label="Encoder route"
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
# RIGHT PANEL - LOG TABLE
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

table_frame.columnconfigure(0, weight=1)
table_frame.rowconfigure(1, weight=1)


table_title = ttk.Label(
    table_frame,
    text="ENCODER COORDINATE LOG",
    font=("Arial", 14, "bold")
)

table_title.grid(
    row=0,
    column=0,
    pady=5
)


columns = (
    "sample",
    "time",
    "x",
    "y",
    "heading",
    "motion"
)


log_table = ttk.Treeview(
    table_frame,
    columns=columns,
    show="headings",
    height=22
)


log_table.heading("sample", text="#")
log_table.heading("time", text="Time")
log_table.heading("x", text="X (m)")
log_table.heading("y", text="Y (m)")
log_table.heading("heading", text="Heading")
log_table.heading("motion", text="Motion")


log_table.column("sample", width=45, anchor="center")
log_table.column("time", width=65, anchor="center")
log_table.column("x", width=75, anchor="center")
log_table.column("y", width=75, anchor="center")
log_table.column("heading", width=80, anchor="center")
log_table.column("motion", width=90, anchor="center")


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
        "Route follows actual encoder movement.\n"
        "Q/E can change X/Y if the robot really shifts.\n"
        "Only very small encoder noise is ignored."
    ),
    justify="center"
)

table_info.grid(
    row=2,
    column=0,
    pady=10
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
