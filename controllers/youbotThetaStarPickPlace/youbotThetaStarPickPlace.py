"""
youbot_pick_place_theta_star_merged.py

Merged Webots youBot controller:

1. Uses Theta* path planning to move from the robot's current pose to a
   staging point near the wooden box.
2. Uses camera recognition for final box centering / approach / grab.
3. Uses Theta* path planning again to move to a staging point near the cabinet.
4. Uses camera recognition for final cabinet centering / approach / placement.

Keep planner_core.py and factory_map.py in the same controller folder.
Tune BOX_STAGING_WORLD and CABINET_STAGING_WORLD for your world.
"""

from controller import Supervisor, Keyboard
import math
import sys

from planner_core import OccupancyGrid, theta_star
from factory_map import build_factory_map

# ----------------------------------
# Robot Instantiation / Timing Setup
# ----------------------------------
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(timestep)

# -----------------------------
# Base constants
# -----------------------------
WHEEL_RADIUS = 0.05
LX = 0.228
LY = 0.158

MAX_SPEED = 0.3
SPEED_INCREMENT = 0.05
MAX_OMEGA = 0.15

# Fine camera-based motion speeds.
FINE_FORWARD_SPEED = 0.10
FINE_STRAFE_SPEED = 0.08
FINE_TURN_SPEED = 0.08

# Set to False if console output becomes too noisy.
DEBUG_SENSOR_PRINTS = False
DEBUG_NAV_PRINTS = True
DEBUG_DRAW_MAP_ON_START = False  # keep False inside Webots unless matplotlib is installed

# -----------------------------
# Recognition / pick-and-place tuning
# -----------------------------
BOX_MODEL = "wooden box"
CABINET_MODEL = "cabinet"

BOX_CENTER_TOLERANCE_PX = 3
BOX_RECENTER_TOLERANCE_PX = 15
BOX_APPROACH_DISTANCE = 0.18
BOX_FOUND_BACKUP_SECONDS = 10

CABINET_CENTER_TOLERANCE_PX = 15
CABINET_RECENTER_TOLERANCE_PX = 25
CABINET_APPROACH_DISTANCE = 0.25

CABINET_PLACE_HEIGHT = 5  # same numeric value as ARM_FRONT_PLATE
CABINET_INSERT_SECONDS = 0.8
CABINET_BACK_OUT_SECONDS = 1.0

# -----------------------------
# Theta* navigation targets
# -----------------------------
def deg_to_rad(degrees):
    return math.radians(degrees)

# Each navigation target may be either:
#   (x, y)
# or:
#   (x, y, final_yaw_radians)
#
# Yaw convention for this controller:
#   0 deg    -> face Webots/planner +X
#   90 deg   -> face Webots/planner +Y
#   -90 deg  -> face Webots/planner -Y
#   180 deg  -> face Webots/planner -X
#
# B: staging point near the box. The map currently marks a box/work-area near
# (1.90, -3.80), so this default stops to the side of it. Tune as needed.


BOX_STAGING_WORLD = (-0.865, -7.77, deg_to_rad(-90.0))

# C: staging point near the cabinet. Tune this to the cabinet approach location
# and final facing direction in your factory world.
CABINET_STAGING_WORLD = (-5.5, -8.16, deg_to_rad(-180.0))

NAV_GOAL_TOLERANCE = 0.18
NAV_WAYPOINT_TOLERANCE = 0.4
NAV_FINAL_YAW_TOLERANCE = deg_to_rad(45.0)
NAV_FINAL_YAW_K = 0.8

GRID_WIDTH = 200
GRID_HEIGHT = 164
GRID_RESOLUTION = 0.10
GRID_ORIGIN_X = -10.0
GRID_ORIGIN_Y = -12.5
OBSTACLE_INFLATION_M = 0.7

# -----------------------------
# Arm State Enumerations
# -----------------------------
ARM_BACK_PLATE_LOW = 0
ARM_BACK_PLATE_HIGH = 1
ARM_RESET = 2
ARM_FRONT_CARDBOARD_BOX = 3
ARM_HANOI_PREPARE = 4
ARM_FRONT_PLATE = 5
ARM_FRONT_FLOOR = 6
ARM_BACK_PLATE_GRAB = 7
ARM_MAX_HEIGHT = 8

ARM_BACK_RIGHT = 0
ARM_RIGHT = 1
ARM_FRONT_RIGHT = 2
ARM_FRONT = 3
ARM_FRONT_LEFT = 4
ARM_LEFT = 5
ARM_BACK_LEFT = 6
ARM_MAX_SIDE = 7

# -----------------------------
# High-level sequence states
# -----------------------------
TASK_IDLE = 0
TASK_NAV_TO_BOX = 1
TASK_FINE_PICK_BOX = 2
TASK_NAV_TO_CABINET = 3
TASK_FINE_PLACE_CABINET = 4
TASK_DONE = 5
TASK_FAILED = 6

# Fine visual-servo states.
FINE_SEARCH = 0
FINE_CENTER = 1
FINE_APPROACH = 2
FINE_ACT = 3
FINE_DONE = 4

sequence_enabled = True
current_task = TASK_NAV_TO_BOX
box_fine_state = FINE_SEARCH
cabinet_fine_state = FINE_SEARCH

# -----------------------------
# Device Initialization
# -----------------------------
def safe_get_device(name):
    """Get an optional Webots device without crashing the controller."""
    try:
        return robot.getDevice(name)
    except Exception:
        return None


wheel_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
wheels = []

for name in wheel_names:
    motor = robot.getDevice(name)
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)
    wheels.append(motor)

arm_names = ["arm1", "arm2", "arm3", "arm4", "arm5"]
arm_motors = []

for name in arm_names:
    motor = robot.getDevice(name)
    arm_motors.append(motor)

# This matches the original C arm behavior where arm2 is slower.
arm_motors[1].setVelocity(0.5)

gripper = robot.getDevice("finger::left")
gripper.setVelocity(0.03)

accel = None
camera = None

display = safe_get_device("display")
if display is not None:
    display_width = display.getWidth()
    display_height = display.getHeight()
else:
    display_width = 0
    display_height = 0

self_node = robot.getSelf()

# -------------------------------------
# Controller State & Initial Conditions
# -------------------------------------
robot_vx = 0.0
robot_vy = 0.0
robot_omega = 0.0

current_height = ARM_RESET
current_orientation = ARM_FRONT

log_lines = []
MAX_LOG_LINES = 12

# Navigation state.
grid_map = None
active_goal_world = None     # XY only: (x, y)
active_goal_yaw = None       # optional final yaw in radians
active_goal_label = ""
path_world = []
current_waypoint_index = 0
path_planned = False

# -----------------------------
# Utility / logging
# -----------------------------
def clamp(value, low, high):
    return max(low, min(high, value))


def step_robot():
    if robot.step(timestep) == -1:
        sys.exit(0)


def passive_wait(seconds):
    start_time = robot.getTime()
    while robot.getTime() < start_time + seconds:
        step_robot()


def log(msg):
    print(msg)
    log_lines.append(str(msg))
    if len(log_lines) > MAX_LOG_LINES:
        log_lines.pop(0)


def draw_log_overlay():
    if display is None:
        return

    display.setColor(0x000000)
    display.fillRectangle(0, 0, display_width, display_height)

    display.setColor(0x00FF00)
    y = 10
    for line in log_lines:
        display.drawText(str(line), 10, y)
        y += 15


def display_helper_message():
    log("\n\nControl commands:")
    log(" Arrows:         Manual move")
    log(" Page Up/Down:   Manual rotate")
    log(" +/-:            (Un)grip")
    log(" Shift + arrows: Handle the arm")
    log(" Space/End:      Reset robot and restart sequence")
    log(" A:              Toggle full autonomous sequence")
    log(" B:              Restart full sequence")
    log(" D:              Run old scripted demo behavior")

# -----------------------------
# Pose sensing [Supervisor-based]
# -----------------------------
def get_robot_pose():
    """
    Returns robot pose as (x, y, yaw).

    The Theta* controller mapped planner x <- Webots X and planner y <- Webots Y.
    This assumes the same factory world convention.
    """
    position = self_node.getPosition()
    orientation = self_node.getOrientation()

    x = position[0]
    y = position[1]

    # For this world, yaw is rotation about Webots Z.
    yaw = math.atan2(orientation[3], orientation[0])
    return x, y, yaw


def angle_wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

# -----------------------------
# Base control
# -----------------------------
def apply_base_speeds(vx, vy, omega):
    """Exact kinematic mapping from base.c."""
    k = LX + LY

    w1 = (vx + vy + k * omega) / WHEEL_RADIUS
    w2 = (vx - vy - k * omega) / WHEEL_RADIUS
    w3 = (vx - vy + k * omega) / WHEEL_RADIUS
    w4 = (vx + vy - k * omega) / WHEEL_RADIUS

    wheels[0].setVelocity(w1)
    wheels[1].setVelocity(w2)
    wheels[2].setVelocity(w3)
    wheels[3].setVelocity(w4)


def base_move(vx, vy, omega):
    apply_base_speeds(vx, vy, omega)
    if DEBUG_NAV_PRINTS:
        print(f"Speeds: vx={vx:.2f} [m/s], vy={vy:.2f} [m/s], omega={omega:.2f} [rad/s]")


def base_reset():
    global robot_vx, robot_vy, robot_omega
    robot_vx = 0.0
    robot_vy = 0.0
    robot_omega = 0.0
    apply_base_speeds(robot_vx, robot_vy, robot_omega)


def base_forwards_increment():
    global robot_vx
    robot_vx += SPEED_INCREMENT
    robot_vx = clamp(robot_vx, -MAX_SPEED, MAX_SPEED)
    base_move(robot_vx, robot_vy, robot_omega)


def base_backwards_increment():
    global robot_vx
    robot_vx -= SPEED_INCREMENT
    robot_vx = clamp(robot_vx, -MAX_SPEED, MAX_SPEED)
    base_move(robot_vx, robot_vy, robot_omega)


def base_strafe_left_increment():
    global robot_vy
    robot_vy += SPEED_INCREMENT
    robot_vy = clamp(robot_vy, -MAX_SPEED, MAX_SPEED)
    base_move(robot_vx, robot_vy, robot_omega)


def base_strafe_right_increment():
    global robot_vy
    robot_vy -= SPEED_INCREMENT
    robot_vy = clamp(robot_vy, -MAX_SPEED, MAX_SPEED)
    base_move(robot_vx, robot_vy, robot_omega)


def base_turn_left_increment():
    global robot_omega
    robot_omega += SPEED_INCREMENT
    robot_omega = clamp(robot_omega, -MAX_SPEED, MAX_SPEED)
    base_move(robot_vx, robot_vy, robot_omega)


def base_turn_right_increment():
    global robot_omega
    robot_omega -= SPEED_INCREMENT
    robot_omega = clamp(robot_omega, -MAX_SPEED, MAX_SPEED)
    base_move(robot_vx, robot_vy, robot_omega)


def base_forwards():
    base_move(MAX_SPEED, 0.0, 0.0)


def base_backwards():
    base_move(-MAX_SPEED, 0.0, 0.0)


def base_strafe_left():
    base_move(0.0, MAX_SPEED, 0.0)


def base_strafe_right():
    base_move(0.0, -MAX_SPEED, 0.0)


def base_turn_left():
    base_move(0.0, 0.0, MAX_SPEED)


def base_turn_right():
    base_move(0.0, 0.0, -MAX_SPEED)

# -----------------------------
# Gripper control
# -----------------------------
def gripper_grip():
    gripper.setPosition(0.0)


def gripper_release():
    gripper.setPosition(0.025)

# -----------------------------
# Arm control
# -----------------------------
def arm_set_height(height):
    global current_height

    if height == ARM_FRONT_FLOOR:
        arm_motors[1].setPosition(-0.97)
        arm_motors[2].setPosition(-1.55)
        arm_motors[3].setPosition(-0.61)
        arm_motors[4].setPosition(0.0)

    elif height == ARM_FRONT_PLATE:
        arm_motors[1].setPosition(-0.62)
        arm_motors[2].setPosition(-0.98)
        arm_motors[3].setPosition(-1.53)
        arm_motors[4].setPosition(0.0)

    elif height == ARM_FRONT_CARDBOARD_BOX:
        arm_motors[1].setPosition(0.0)
        arm_motors[2].setPosition(-0.97)
        arm_motors[3].setPosition(-1.21)
        arm_motors[4].setPosition(0.0)

    elif height == ARM_RESET:
        arm_motors[1].setPosition(1.57)
        arm_motors[2].setPosition(-2.635)
        arm_motors[3].setPosition(1.78)
        arm_motors[4].setPosition(0.0)

    elif height == ARM_BACK_PLATE_HIGH:
        arm_motors[1].setPosition(0.678)
        arm_motors[2].setPosition(0.682)
        arm_motors[3].setPosition(1.74)
        arm_motors[4].setPosition(0.0)

    elif height == ARM_BACK_PLATE_LOW:
        arm_motors[1].setPosition(0.92)
        arm_motors[2].setPosition(0.42)
        arm_motors[3].setPosition(1.78)
        arm_motors[4].setPosition(0.0)

    elif height == ARM_HANOI_PREPARE:
        arm_motors[1].setPosition(-0.4)
        arm_motors[2].setPosition(-1.2)
        arm_motors[3].setPosition(-math.pi / 2.0)
        arm_motors[4].setPosition(math.pi / 2.0)

    elif height == ARM_BACK_PLATE_GRAB:
        arm_motors[1].setPosition(0.92)
        arm_motors[2].setPosition(0.42)
        arm_motors[3].setPosition(1.58)
        arm_motors[4].setPosition(0.0)

    else:
        log(f"arm_set_height() called with wrong argument: {height}")
        return

    current_height = height


def arm_set_orientation(orientation):
    global current_orientation

    if orientation == ARM_BACK_LEFT:
        arm_motors[0].setPosition(-2.949)
    elif orientation == ARM_LEFT:
        arm_motors[0].setPosition(-math.pi / 2.0)
    elif orientation == ARM_FRONT_LEFT:
        arm_motors[0].setPosition(-0.2)
    elif orientation == ARM_FRONT:
        arm_motors[0].setPosition(0.0)
    elif orientation == ARM_FRONT_RIGHT:
        arm_motors[0].setPosition(0.2)
    elif orientation == ARM_RIGHT:
        arm_motors[0].setPosition(math.pi / 2.0)
    elif orientation == ARM_BACK_RIGHT:
        arm_motors[0].setPosition(2.949)
    else:
        log(f"arm_set_orientation() called with wrong argument: {orientation}")
        return

    current_orientation = orientation


def arm_reset():
    arm_set_height(ARM_RESET)
    arm_set_orientation(ARM_FRONT)


def arm_increase_height():
    global current_height
    new_height = current_height + 1

    if new_height >= ARM_MAX_HEIGHT:
        new_height = ARM_MAX_HEIGHT - 1

    if new_height == ARM_FRONT_FLOOR and current_orientation in (ARM_BACK_LEFT, ARM_BACK_RIGHT):
        new_height = current_height

    arm_set_height(new_height)


def arm_decrease_height():
    global current_height
    new_height = current_height - 1
    if new_height < 0:
        new_height = 0
    arm_set_height(new_height)


def arm_increase_orientation():
    global current_orientation
    new_orientation = current_orientation + 1

    if new_orientation >= ARM_MAX_SIDE:
        new_orientation = ARM_MAX_SIDE - 1

    if new_orientation == ARM_BACK_LEFT and current_height == ARM_FRONT_FLOOR:
        new_orientation = current_orientation

    arm_set_orientation(new_orientation)


def arm_decrease_orientation():
    global current_orientation
    new_orientation = current_orientation - 1

    if new_orientation < 0:
        new_orientation = 0

    if new_orientation == ARM_BACK_RIGHT and current_height == ARM_FRONT_FLOOR:
        new_orientation = current_orientation

    arm_set_orientation(new_orientation)

# -----------------------------
# Camera / accelerometer setup
# -----------------------------
def initialize_optional_sensors():
    global accel, camera

    accel = safe_get_device("accel")
    if accel is not None:
        accel.enable(timestep)
        log("accel found")
    else:
        log("accel not found")

    camera = safe_get_device("camera")
    if camera is None:
        log("camera not found")
        return

    log("camera found")
    camera.enable(timestep)

    if hasattr(camera, "recognitionEnable"):
        camera.recognitionEnable(timestep)
    else:
        log("Warning: camera does not expose recognitionEnable().")

    for _ in range(3):
        step_robot()

    if camera.getImage():
        log("camera image OK after warmup")
    else:
        log("camera image still null after warmup")

# -----------------------------
# Camera recognition helpers
# -----------------------------
def get_object_model(obj):
    """Safe wrapper around Webots recognition object's getModel()."""
    if hasattr(obj, "getModel"):
        try:
            model = obj.getModel()
            if model is None:
                return ""
            return model
        except UnicodeDecodeError:
            raw = getattr(obj, "_model", b"")
            if isinstance(raw, bytes):
                return raw.decode("utf-8", errors="ignore")
            return ""
        except Exception as exc:
            log(f"Could not read recognition model: {exc}")
            return ""

    try:
        model = getattr(obj, "model", "")
        if model is None:
            return ""
        return str(model)
    except Exception:
        return ""


def get_object_id(obj):
    if hasattr(obj, "getId"):
        return obj.getId()
    return getattr(obj, "id", None)


def get_object_position_on_image(obj):
    if hasattr(obj, "getPositionOnImage"):
        return obj.getPositionOnImage()
    return getattr(obj, "position_on_image", [0, 0])


def get_object_size_on_image(obj):
    if hasattr(obj, "getSizeOnImage"):
        return obj.getSizeOnImage()
    return getattr(obj, "size_on_image", [0, 0])


def get_object_position(obj):
    if hasattr(obj, "getPosition"):
        return obj.getPosition()
    return getattr(obj, "position", [0.0, 0.0, 0.0])


def get_recognition_objects():
    if camera is None:
        return []

    try:
        return camera.getRecognitionObjects()
    except Exception as exc:
        log(f"Could not read camera recognition objects: {exc}")
        return []


def find_recognized_model(objects, target_model):
    for obj in objects:
        if get_object_model(obj) == target_model:
            return obj
    return None


def find_box(objects):
    return find_recognized_model(objects, BOX_MODEL)


def find_cabinet(objects):
    return find_recognized_model(objects, CABINET_MODEL)


def print_recognition_debug(objects):
    print(f"Detected objects: {len(objects)}")

    for i, obj in enumerate(objects):
        pos_img = get_object_position_on_image(obj)
        size_img = get_object_size_on_image(obj)
        pos = get_object_position(obj)
        model = get_object_model(obj)

        print(f"Object {i}:")
        print(f"  id: {get_object_id(obj)}")
        print(f"  model: {model if model else '<no valid model>'}")
        print(f"  pos on image: {pos_img[0]} {pos_img[1]}")
        print(f"  size on image: {size_img[0]} {size_img[1]}")
        print(f"  relative position: {pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f}")


def print_accelerometer_debug():
    if accel is None:
        return

    try:
        a = accel.getValues()
        print(f"Accel: x={a[0]:f} y={a[1]:f} z={a[2]:f}")
    except Exception as exc:
        print(f"Could not read accelerometer: {exc}")

# -----------------------------
# Theta* map / route-following layer
# -----------------------------
def setup_grid_map():
    grid = OccupancyGrid(
        width=GRID_WIDTH,
        height=GRID_HEIGHT,
        resolution=GRID_RESOLUTION,
        origin_x=GRID_ORIGIN_X,
        origin_y=GRID_ORIGIN_Y,
    )

    build_factory_map(grid)
    grid.inflate_obstacles(OBSTACLE_INFLATION_M)
    return grid


def nearest_free_cell(cell, max_radius=20):
    """Return cell if free, otherwise nearest free cell within max_radius."""
    if grid_map is None:
        return cell

    ci, cj = cell
    if grid_map.is_free(ci, cj):
        return cell

    for r in range(1, max_radius + 1):
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if abs(di) != r and abs(dj) != r:
                    continue
                ni, nj = ci + di, cj + dj
                if grid_map.is_free(ni, nj):
                    return ni, nj

    return cell


def split_navigation_goal(goal_world):
    """
    Accept either (x, y) or (x, y, yaw).
    Returns ((x, y), yaw_or_None).
    """
    if len(goal_world) >= 3:
        return (goal_world[0], goal_world[1]), goal_world[2]
    return (goal_world[0], goal_world[1]), None


def start_navigation(goal_world, label):
    global active_goal_world, active_goal_yaw, active_goal_label
    global path_world, current_waypoint_index, path_planned

    active_goal_world, active_goal_yaw = split_navigation_goal(goal_world)
    active_goal_label = label
    path_world = []
    current_waypoint_index = 0
    path_planned = False

    if active_goal_yaw is None:
        log(f"Navigation target set: {label} at {active_goal_world}")
    else:
        log(
            f"Navigation target set: {label} at {active_goal_world}, "
            f"final yaw={math.degrees(active_goal_yaw):.1f} deg"
        )


def plan_active_path():
    global path_world, current_waypoint_index, path_planned

    if active_goal_world is None:
        log("No active navigation goal.")
        path_planned = True
        return False

    x, y, _ = get_robot_pose()
    raw_start_cell = grid_map.world_to_grid(x, y)
    raw_goal_cell = grid_map.world_to_grid(active_goal_world[0], active_goal_world[1])

    start_cell = nearest_free_cell(raw_start_cell)
    goal_cell = nearest_free_cell(raw_goal_cell)

    if start_cell != raw_start_cell:
        log(f"Adjusted blocked start cell {raw_start_cell} -> {start_cell}")

    if goal_cell != raw_goal_cell:
        adjusted_goal = grid_map.grid_to_world(goal_cell[0], goal_cell[1])
        log(f"Adjusted blocked goal cell {raw_goal_cell} -> {goal_cell}, world={adjusted_goal}")

    path_cells = theta_star(grid_map, start_cell, goal_cell)

    if not path_cells:
        log(f"Theta* failed: no path to {active_goal_label}.")
        base_reset()
        path_planned = True
        return False

    path_world = [grid_map.grid_to_world(i, j) for i, j in path_cells]
    current_waypoint_index = 1 if len(path_world) > 1 else 0
    path_planned = True

    log(f"Theta* path to {active_goal_label}: {len(path_world)} waypoints")
    for p in path_world:
        print(f"  {p}")

    return True


def final_yaw_alignment_step(yaw):
    """
    Rotate in place after XY arrival until the final requested yaw is reached.
    Return True when final orientation is reached.
    """
    if active_goal_yaw is None:
        base_reset()
        return True

    yaw_error = angle_wrap(active_goal_yaw - yaw)

    if abs(yaw_error) <= NAV_FINAL_YAW_TOLERANCE:
        base_reset()
        log(
            f"Arrived at {active_goal_label} with yaw "
            f"{math.degrees(yaw):.1f} deg."
        )
        return True

    omega = clamp(NAV_FINAL_YAW_K * yaw_error, -MAX_OMEGA, MAX_OMEGA)

    if DEBUG_NAV_PRINTS:
        print(
            f"final yaw align: current={math.degrees(yaw):.1f} deg "
            f"target={math.degrees(active_goal_yaw):.1f} deg "
            f"error={math.degrees(yaw_error):.1f} deg omega={omega:.2f}"
        )

    base_move(0.0, 0.0, omega)
    return False


def navigation_step():
    """
    Return: 'running', 'arrived', or 'failed'.
    """
    global current_waypoint_index

    if grid_map is None:
        log("No grid map available.")
        base_reset()
        return "failed"

    if not path_planned:
        if not plan_active_path():
            return "failed"

    x, y, yaw = get_robot_pose()

    if current_waypoint_index >= len(path_world):
        if final_yaw_alignment_step(yaw):
            return "arrived"
        return "running"

    final_goal_x, final_goal_y = path_world[-1]
    final_distance = math.hypot(final_goal_x - x, final_goal_y - y)
    if final_distance < NAV_GOAL_TOLERANCE:
        if final_yaw_alignment_step(yaw):
            return "arrived"
        return "running"

    target_x, target_y = path_world[current_waypoint_index]
    dx = target_x - x
    dy = target_y - y
    distance = math.hypot(dx, dy)

    if distance < NAV_WAYPOINT_TOLERANCE:
        current_waypoint_index += 1
        return "running"

    desired_heading = math.atan2(dy, dx)
    heading_error = angle_wrap(desired_heading - yaw)

    k_forward = 0.5
    k_turn = 0.8

    # Keep the same conservative behavior as the Theta* controller: turn first,
    # then drive forward, instead of relying on lateral Mecanum strafing.
    if abs(heading_error) > 0.25:
        vx = 0.0
    else:
        vx = clamp(k_forward * distance, 0.0, MAX_SPEED)

    vy = 0.0
    omega = clamp(k_turn * heading_error, -MAX_OMEGA, MAX_OMEGA)

    if DEBUG_NAV_PRINTS:
        print(
            f"nav={active_goal_label} pose=({x:.2f}, {y:.2f}, yaw={yaw:.2f}) "
            f"target=({target_x:.2f}, {target_y:.2f}) "
            f"dist={distance:.2f} heading_error={heading_error:.2f} "
            f"cmd=({vx:.2f}, {vy:.2f}, {omega:.2f})"
        )

    base_move(vx, vy, omega)
    return "running"

# -----------------------------
# Manipulation behaviors
# -----------------------------
def grab_box_sequence():
    """Run after visual approach has stopped in front of the box."""
    base_reset()
    passive_wait(0.3)

    gripper_release()
    arm_set_orientation(ARM_FRONT)
    arm_set_height(ARM_FRONT_CARDBOARD_BOX)
    passive_wait(4.0)

    gripper_grip()
    passive_wait(1.0)

    arm_reset()
    passive_wait(2.0)
    base_reset()


def back_up_after_grabbing_box():
    log("Box grabbed. Backing up slightly before Theta* navigation to cabinet.")
    base_reset()
    passive_wait(0.2)

    base_backwards()
    passive_wait(BOX_FOUND_BACKUP_SECONDS)

    base_reset()
    passive_wait(0.2)


def place_box_in_cabinet_sequence():
    """Run after the robot has visually centered and approached the cabinet."""
    base_reset()

    arm_set_orientation(ARM_FRONT)
    arm_set_height(CABINET_PLACE_HEIGHT)
    passive_wait(2.0)

    base_forwards()
    passive_wait(CABINET_INSERT_SECONDS)
    base_reset()
    passive_wait(0.5)

    gripper_release()
    passive_wait(1.0)

    base_backwards()
    passive_wait(CABINET_BACK_OUT_SECONDS)
    base_reset()

    arm_reset()
    passive_wait(1.0)


def automatic_behavior():
    """Original scripted demo behavior, kept for testing/manual fallback."""
    passive_wait(2.0)
    gripper_release()
    arm_set_height(ARM_FRONT_CARDBOARD_BOX)
    passive_wait(4.0)
    gripper_grip()
    passive_wait(4.0)
    arm_set_height(ARM_BACK_PLATE_LOW)
    passive_wait(3.0)
    gripper_release()
    passive_wait(1.0)
    arm_reset()
    base_strafe_left()
    passive_wait(5.0)
    gripper_grip()
    base_reset()
    passive_wait(1.0)
    base_turn_left()
    passive_wait(1.0)
    base_reset()
    gripper_release()
    arm_set_height(ARM_BACK_PLATE_LOW)
    passive_wait(3.0)
    gripper_grip()
    passive_wait(1.0)
    arm_set_height(ARM_RESET)
    passive_wait(2.0)
    arm_set_height(ARM_FRONT_PLATE)
    arm_set_orientation(ARM_RIGHT)
    passive_wait(4.0)
    arm_set_height(ARM_FRONT_FLOOR)
    passive_wait(2.0)
    gripper_release()
    passive_wait(1.0)
    arm_set_height(ARM_FRONT_PLATE)
    passive_wait(2.0)
    arm_set_height(ARM_RESET)
    passive_wait(2.0)
    arm_reset()
    gripper_grip()
    passive_wait(2.0)

# -----------------------------
# Fine camera-based alignment routines
# -----------------------------
def center_target_on_image(error_x, tolerance_px):
    if abs(error_x) <= tolerance_px:
        base_reset()
        return True

    if error_x < 0:
        base_move(0.0, FINE_STRAFE_SPEED, 0.0)
    else:
        base_move(0.0, -FINE_STRAFE_SPEED, 0.0)

    return False


def approach_target(error_x, rel_x, rel_z, recenter_tolerance_px, stop_distance, label):
    if abs(error_x) > recenter_tolerance_px:
        log(f"Recentering on {label}")
        return "recenter"

    # This intentionally uses rel_x because your working box approach used pos[0].
    # If cabinet approach behaves backwards or never arrives, test rel_z instead.
    if rel_x > stop_distance:
        log(f"Approaching {label}, rel_x={rel_x:.3f}, rel_z={rel_z:.3f}, stop={stop_distance:.3f}")
        base_move(FINE_FORWARD_SPEED, 0.0, 0.0)
        return "approaching"

    base_reset()
    return "arrived"


def fine_pick_box_step(objects):
    """Camera-based final search/center/approach/grab. Return 'running' or 'done'."""
    global box_fine_state

    if camera is None:
        log("Cannot pick box: no camera.")
        return "failed"

    image_center_x = camera.getWidth() // 2
    box = find_box(objects)

    if box is None:
        box_fine_state = FINE_SEARCH
        log("No box detected during final pickup alignment")
        base_move(0.0, 0.0, FINE_TURN_SPEED)
        return "running"

    pos_img = get_object_position_on_image(box)
    pos = get_object_position(box)

    x_img = int(pos_img[0])
    error_x = x_img - image_center_x
    rel_x = pos[0]
    rel_z = pos[2]

    print(
        f"Box img_x={x_img} error_x={error_x} "
        f"rel=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
    )

    if box_fine_state == FINE_SEARCH:
        box_fine_state = FINE_CENTER

    elif box_fine_state == FINE_CENTER:
        if center_target_on_image(error_x, BOX_CENTER_TOLERANCE_PX):
            box_fine_state = FINE_APPROACH

    elif box_fine_state == FINE_APPROACH:
        result = approach_target(
            error_x,
            rel_x,
            rel_z,
            BOX_RECENTER_TOLERANCE_PX,
            BOX_APPROACH_DISTANCE,
            "box",
        )

        if result == "recenter":
            box_fine_state = FINE_CENTER
        elif result == "arrived":
            box_fine_state = FINE_ACT

    elif box_fine_state == FINE_ACT:
        grab_box_sequence()
        back_up_after_grabbing_box()
        box_fine_state = FINE_DONE
        return "done"

    elif box_fine_state == FINE_DONE:
        base_reset()
        return "done"

    return "running"


def fine_place_cabinet_step(objects):
    """Camera-based final search/center/approach/place. Return 'running' or 'done'."""
    global cabinet_fine_state

    if camera is None:
        log("Cannot place box: no camera.")
        return "failed"

    image_center_x = camera.getWidth() // 2
    cabinet = find_cabinet(objects)

    if cabinet is None:
        cabinet_fine_state = FINE_SEARCH
        log("No cabinet detected during final placement alignment")
        base_move(0.0, 0.0, FINE_TURN_SPEED)
        return "running"

    pos_img = get_object_position_on_image(cabinet)
    pos = get_object_position(cabinet)

    x_img = int(pos_img[0])
    error_x = x_img - image_center_x
    rel_x = pos[0]
    rel_z = pos[2]

    print(
        f"Cabinet img_x={x_img} error_x={error_x} "
        f"rel=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
    )

    if cabinet_fine_state == FINE_SEARCH:
        cabinet_fine_state = FINE_CENTER

    elif cabinet_fine_state == FINE_CENTER:
        if center_target_on_image(error_x, CABINET_CENTER_TOLERANCE_PX):
            cabinet_fine_state = FINE_APPROACH

    elif cabinet_fine_state == FINE_APPROACH:
        result = approach_target(
            error_x,
            rel_x,
            rel_z,
            CABINET_RECENTER_TOLERANCE_PX,
            CABINET_APPROACH_DISTANCE,
            "cabinet",
        )

        if result == "recenter":
            cabinet_fine_state = FINE_CENTER
        elif result == "arrived":
            cabinet_fine_state = FINE_ACT

    elif cabinet_fine_state == FINE_ACT:
        place_box_in_cabinet_sequence()
        cabinet_fine_state = FINE_DONE
        return "done"

    elif cabinet_fine_state == FINE_DONE:
        base_reset()
        return "done"

    return "running"

# -----------------------------
# High-level merged task sequence
# -----------------------------
def start_full_sequence():
    global sequence_enabled, current_task, box_fine_state, cabinet_fine_state

    sequence_enabled = True
    current_task = TASK_NAV_TO_BOX
    box_fine_state = FINE_SEARCH
    cabinet_fine_state = FINE_SEARCH
    start_navigation(BOX_STAGING_WORLD, "box staging point")
    log("Full sequence started: Theta* -> box -> Theta* -> cabinet")


def stop_full_sequence():
    global sequence_enabled, current_task
    sequence_enabled = False
    current_task = TASK_IDLE
    base_reset()
    log("Full sequence stopped.")


def automatic_sequence_step(objects):
    global current_task

    if not sequence_enabled:
        return

    if current_task == TASK_IDLE:
        base_reset()
        return

    if current_task == TASK_NAV_TO_BOX:
        status = navigation_step()
        if status == "arrived":
            current_task = TASK_FINE_PICK_BOX
            log("Reached box staging point. Starting visual box pickup.")
        elif status == "failed":
            current_task = TASK_FAILED
        return

    if current_task == TASK_FINE_PICK_BOX:
        status = fine_pick_box_step(objects)
        if status == "done":
            current_task = TASK_NAV_TO_CABINET
            start_navigation(CABINET_STAGING_WORLD, "cabinet staging point")
            log("Box picked. Starting Theta* navigation to cabinet.")
        elif status == "failed":
            current_task = TASK_FAILED
        return

    if current_task == TASK_NAV_TO_CABINET:
        status = navigation_step()
        if status == "arrived":
            current_task = TASK_FINE_PLACE_CABINET
            log("Reached cabinet staging point. Starting visual cabinet placement.")
        elif status == "failed":
            current_task = TASK_FAILED
        return

    if current_task == TASK_FINE_PLACE_CABINET:
        status = fine_place_cabinet_step(objects)
        if status == "done":
            current_task = TASK_DONE
            log("Pick-and-place sequence complete.")
        elif status == "failed":
            current_task = TASK_FAILED
        return

    if current_task == TASK_DONE:
        base_reset()
        return

    if current_task == TASK_FAILED:
        base_reset()
        log("Sequence failed. Check map target, object recognition, or blocked path.")
        return

# -----------------------------
# Keyboard handling
# -----------------------------
def handle_keyboard(key, previous_key):
    global sequence_enabled, current_task

    if key < 0 or key == previous_key:
        return

    if key == Keyboard.UP:
        base_forwards_increment()

    elif key == Keyboard.DOWN:
        base_backwards_increment()

    elif key == Keyboard.LEFT:
        base_strafe_left_increment()

    elif key == Keyboard.RIGHT:
        base_strafe_right_increment()

    elif key == Keyboard.PAGEUP:
        base_turn_left_increment()

    elif key == Keyboard.PAGEDOWN:
        base_turn_right_increment()

    elif key == Keyboard.END or key == ord(" "):
        log("Reset")
        base_reset()
        arm_reset()
        gripper_release()
        start_full_sequence()

    elif key in (ord("+"), 388, 65585):
        log("Grip")
        gripper_grip()

    elif key in (ord("-"), 390):
        log("Ungrip")
        gripper_release()

    elif key in (Keyboard.SHIFT + Keyboard.UP, 332):
        log("Increase arm height")
        arm_increase_height()

    elif key in (Keyboard.SHIFT + Keyboard.DOWN, 326):
        log("Decrease arm height")
        arm_decrease_height()

    elif key in (Keyboard.SHIFT + Keyboard.RIGHT, 330):
        log("Increase arm orientation")
        arm_increase_orientation()

    elif key in (Keyboard.SHIFT + Keyboard.LEFT, 328):
        log("Decrease arm orientation")
        arm_decrease_orientation()

    elif key in (ord("a"), ord("A")):
        if sequence_enabled:
            stop_full_sequence()
        else:
            start_full_sequence()

    elif key in (ord("b"), ord("B")):
        start_full_sequence()

    elif key in (ord("d"), ord("D")):
        log("Running old scripted demo behavior")
        automatic_behavior()

    else:
        log("Wrong keyboard input")

# -----------------------------
# Initial setup
# -----------------------------
base_reset()
arm_reset()
gripper_release()

initialize_optional_sensors()

grid_map = setup_grid_map()

# Map visualization is intentionally disabled in the Webots controller.
# Webots' bundled Python often does not include matplotlib/numpy. Use
# factory_map.visualize_factory_map() only in a normal desktop Python install.
if DEBUG_DRAW_MAP_ON_START:
    log("DEBUG_DRAW_MAP_ON_START is True, but map drawing is disabled in Webots-safe mode.")

# Match the existing controller's warmup behavior, but do not over-delay.
passive_wait(1.0)

if len(sys.argv) > 1 and sys.argv[1] == "demo":
    automatic_behavior()

start_full_sequence()
display_helper_message()

# -----------------------------
# Main loop
# -----------------------------
previous_key = -1

while robot.step(timestep) != -1:
    objects = get_recognition_objects()

    if sequence_enabled:
        automatic_sequence_step(objects)

    if DEBUG_SENSOR_PRINTS:
        print_recognition_debug(objects)
        print_accelerometer_debug()

    key = keyboard.getKey()
    handle_keyboard(key, previous_key)
    previous_key = key

    draw_log_overlay()
