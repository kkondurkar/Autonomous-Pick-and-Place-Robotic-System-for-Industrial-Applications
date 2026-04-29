"""youbot_thetaStar controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor, Keyboard
import math
import heapq

# ----------------------------------
# Robot Instantiation / Timing Setup
# ----------------------------------
# robot = Robot()---->replaced with:
robot = Supervisor()

# get the time step of the current world.
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
ARM_MAX_HEIGHT = 7

ARM_BACK_RIGHT = 0
ARM_RIGHT = 1
ARM_FRONT_RIGHT = 2
ARM_FRONT = 3
ARM_FRONT_LEFT = 4
ARM_LEFT = 5
ARM_BACK_LEFT = 6
ARM_MAX_SIDE = 7

# -----------------------------
# Device Initialization
# -----------------------------
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

wheel_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
wheels = []

for name in wheel_names:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
    wheels.append(motor)

arm_names = ["arm1", "arm2", "arm3", "arm4", "arm5"]
arm_motors = []

for name in arm_names:
    motor = robot.getDevice(name)
    arm_motors.append(motor)

# This matches up with arm.c (as reference)
arm_motors[1].setVelocity(0.5)  # arm2 slower in original C code

gripper = robot.getDevice("finger::left")
gripper.setVelocity(0.03)

# -------------------------------
# Pose Sensing [Supervisor-based]
# -------------------------------
self_node = robot.getSelf()

def get_robot_pose():
    """
    Returns robot pose as (x, y, yaw)

    Webots uses X-Z as the horizontal ground plane.
    So we map:
      planner x <- Webots x
      planner y <- Webots z
    """
    position = self_node.getPosition()
    orientation = self_node.getOrientation()

    x = position[0]
    y = position[2]

    # For a robot upright on the ground plane, yaw can be extracted from
    # the rotation matrix entries related to X-Z heading.
    yaw = math.atan2(orientation[2], orientation[0])

    return x, y, yaw

# -------------------------------------
# Controller State & Initial Conditions
# -------------------------------------
robot_vx = 0.0
robot_vy = 0.0
robot_omega = 0.0

current_height = ARM_RESET
current_orientation = ARM_FRONT

autonomous_mode = False   # keep manual mode [for now]

# -----------------------------
# Console Helper Message
# -----------------------------
def display_helper_message():
    print("\n\nControl commands:")
    print(" Arrows:         Move the robot")
    print(" Page Up/Down:   Rotate the robot")
    print(" +/-:            (Un)grip")
    print(" Shift + arrows: Handle the arm")
    print(" Space:          Reset")
    print(" A:              Toggle autonomous mode (placeholder)")

display_helper_message()

# -----------------------------
# Base control
# -----------------------------
def clamp(value, low, high):
    return max(low, min(high, value))

def apply_base_speeds(vx, vy, omega):
    """Exact kinematic mapping from base.c"""
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

# -----------------------------
# Gripper control
# -----------------------------
def gripper_grip():
    # gripper.c -> MIN_POS = 0.0
    gripper.setPosition(0.0)

def gripper_release():
    # gripper.c -> MAX_POS = 0.025
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
        arm_motors[2].setPosition(-0.77)
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

    else:
        print("arm_set_height() called with wrong argument")
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
        print("arm_set_orientation() called with wrong argument")
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

    if new_height == ARM_FRONT_FLOOR:
        if current_orientation in (ARM_BACK_LEFT, ARM_BACK_RIGHT):
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

    if new_orientation == ARM_BACK_LEFT:
        if current_height == ARM_FRONT_FLOOR:
            new_orientation = current_orientation

    arm_set_orientation(new_orientation)

def arm_decrease_orientation():
    global current_orientation
    new_orientation = current_orientation - 1

    if new_orientation < 0:
        new_orientation = 0

    if new_orientation == ARM_BACK_RIGHT:
        if current_height == ARM_FRONT_FLOOR:
            new_orientation = current_orientation

    arm_set_orientation(new_orientation)

# -----------------------------
# Autonomous Behavior
# -----------------------------
    """
     This function should:
      1. read current robot pose
      2. compute / follow a Theta* waypoint path
      3. call base_move(vx, vy, omega)
    """
    
def angle_wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def autonomous_step():
    global path_planned, path_world, current_waypoint_index

    x, y, yaw = get_robot_pose()

    if not path_planned:
        start_cell = grid_map.world_to_grid(x, y)
        goal_cell = grid_map.world_to_grid(goal_world[0], goal_world[1])

        path_cells = theta_star(grid_map, start_cell, goal_cell)

        if not path_cells:
            print("Theta* failed: no path found.")
            base_reset()
            path_planned = True
            return

        path_world = [grid_map.grid_to_world(i, j) for i, j in path_cells]
        current_waypoint_index = 0
        path_planned = True

        print("Theta* path:")
        for p in path_world:
            print(p)

    if current_waypoint_index >= len(path_world):
        print("Goal reached.")
        base_reset()
        return

    target_x, target_y = path_world[current_waypoint_index]

    dx = target_x - x
    dy = target_y - y
    distance = math.hypot(dx, dy)

    if distance < 0.15:
        current_waypoint_index += 1
        return

    # Transform target vector from world frame to robot frame.
    x_rel = math.cos(yaw) * dx + math.sin(yaw) * dy
    y_rel = -math.sin(yaw) * dx + math.cos(yaw) * dy

    desired_heading = math.atan2(dy, dx)
    heading_error = angle_wrap(desired_heading - yaw)

    k_xy = 0.8
    k_yaw = 0.8

    vx = clamp(k_xy * x_rel, -MAX_SPEED, MAX_SPEED)
    vy = clamp(k_xy * y_rel, -MAX_SPEED, MAX_SPEED)
    omega = clamp(k_yaw * heading_error, -MAX_SPEED, MAX_SPEED)

    base_move(vx, vy, omega)

# -----------------------------
# Initial Reset State
# -----------------------------
base_reset()
arm_reset()
gripper_release()

# -----------------------------
# Map setup
# -----------------------------
grid_map = OccupancyGrid(
    width=120,
    height=120,
    resolution=0.10,
    origin_x=-6.0,
    origin_y=-6.0
)

# Example rectangular obstacles:
# This requires editing--->Replace these with your actual 
# warehouse shelves/walls:
grid_map.add_rect_obstacle(-2.0, -1.0, -1.5, 3.0)
grid_map.add_rect_obstacle(1.0, -3.0, 1.5, 2.0)
grid_map.add_rect_obstacle(3.0, 1.0, 4.0, 1.5)

# Inflate obstacles so the youBot does not clip corners.
grid_map.inflate_obstacles(0.35)

# Set Goal Node:
goal_world = (4.0, 4.0)
path_world = []
current_waypoint_index = 0
path_planned = False

#planner x = Webots X
#planner y = Webots Z

# -----------------------------
# Main loop
# -----------------------------
previous_key = -1

# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if autonomous_mode:
        autonomous_step()
        continue

    key = keyboard.getKey()

    if key >= 0 and key != previous_key:
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

        elif key == Keyboard.END or key == ord(' '):
            print("Reset")
            base_reset()
            arm_reset()

        elif key == ord('+'):
            print("Grip")
            gripper_grip()

        elif key == ord('-'):
            print("Ungrip")
            gripper_release()

        elif key == ord('A') or key == ord('a'):
            autonomous_mode = not autonomous_mode
            print(f"Autonomous mode: {autonomous_mode}")
        
            if autonomous_mode:
                path_planned = False
                path_world = []
                current_waypoint_index = 0
            else:
                base_reset()
        
        elif key == (Keyboard.SHIFT + Keyboard.UP):
            print("Increase arm height")
            arm_increase_height()

        elif key == (Keyboard.SHIFT + Keyboard.DOWN):
            print("Decrease arm height")
            arm_decrease_height()

        elif key == (Keyboard.SHIFT + Keyboard.RIGHT):
            print("Increase arm orientation")
            arm_increase_orientation()

        elif key == (Keyboard.SHIFT + Keyboard.LEFT):
            print("Decrease arm orientation")
            arm_decrease_orientation()

        else:
            print("Wrong keyboard input")

    previous_key = key

    # Read the sensors (Gabriel's camera / lidar data)
    # Functions to read sensor data:
    
    # Process sensor data here:

    # (Once position in front of "recognized object" reached and proper pose assumed)
    # Enter function calls to send actuator commands to arm here:
        
    pass

# [Enter exit cleanup code here if required: ]
