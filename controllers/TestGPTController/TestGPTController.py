"""Merged youBot controller.

This keeps the existing Python base/arm/gripper/manual-control code and adds the
extra behavior from the C controller:

- accelerometer setup and debug printing
- camera setup and recognition setup
- object-recognition search for model "wooden box"
- search/center/approach/grab state machine
- grab_box_sequence()
- automatic_behavior() demo sequence

Run with controller argument "demo" if you want to execute the scripted demo
sequence before entering the normal loop.
"""

from controller import Robot, Keyboard
import math
import sys

# ----------------------------------
# Robot Instantiation / Timing Setup
# ----------------------------------
robot = Robot()
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

# Set this to False if the console output becomes too noisy.
DEBUG_SENSOR_PRINTS = True

# The box adds load to the arm. These values reduce the chance that the arm
# visually sags/drops after gripping by slowing the move and increasing hold effort.
ARM_MOTOR_VELOCITY = 0.35
ARM_MOTOR_HOLD_TORQUE = 100.0
GRIPPER_VELOCITY = 0.03
GRIPPER_HOLD_FORCE = 20.0

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
# Box pickup state machine modes
# -----------------------------
MODE_SEARCH = 0
MODE_CENTER = 1
MODE_APPROACH = 2
MODE_GRAB = 3
MODE_DONE = 4

mode = MODE_SEARCH
box_pickup_enabled = True

# -----------------------------
# Motor strength helper
# -----------------------------
def try_set_motor_strength(motor, value):
    """Apply holding effort if the Webots motor type supports it."""
    try:
        motor.setAvailableTorque(value)
        return
    except Exception:
        pass

    try:
        motor.setAvailableForce(value)
    except Exception:
        pass

# -----------------------------
# Device Initialization
# -----------------------------
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
    motor.setVelocity(ARM_MOTOR_VELOCITY)
    try_set_motor_strength(motor, ARM_MOTOR_HOLD_TORQUE)
    arm_motors.append(motor)

# This matches the original C arm behavior where arm2 is slower, but also keeps
# the movement gentle enough that the loaded arm is less likely to drop suddenly.
arm_motors[1].setVelocity(ARM_MOTOR_VELOCITY)

gripper = robot.getDevice("finger::left")
gripper.setVelocity(GRIPPER_VELOCITY)
try_set_motor_strength(gripper, GRIPPER_HOLD_FORCE)

# Optional sensors added from the C controller.
accel = None
camera = None

# -------------------------------------
# Controller State & Initial Conditions
# -------------------------------------
robot_vx = 0.0
robot_vy = 0.0
robot_omega = 0.0

current_height = ARM_RESET
current_orientation = ARM_FRONT

autonomous_mode = False  # kept from the original Python file as a placeholder

# -----------------------------
# Utility functions
# -----------------------------
def clamp(value, low, high):
    return max(low, min(high, value))


def step_robot():
    """Equivalent to the C step() helper."""
    if robot.step(timestep) == -1:
        sys.exit(0)


def passive_wait(seconds):
    """Equivalent to the C passive_wait(sec) helper."""
    start_time = robot.getTime()
    while robot.getTime() < start_time + seconds:
        step_robot()


def safe_get_device(name):
    """Get an optional Webots device without crashing the controller."""
    try:
        return robot.getDevice(name)
    except Exception:
        return None

# -----------------------------
# Console Helper Message
# -----------------------------
def display_helper_message():
    print("\n\nControl commands:")
    print(" Arrows:         Move the robot")
    print(" Page Up/Down:   Rotate the robot")
    print(" +/-:            (Un)grip")
    print(" Shift + arrows: Handle the arm")
    print(" Space/End:      Reset")
    print(" B:              Toggle automatic box pickup")
    print(" D:              Run scripted demo behavior")

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

# Non-increment helpers used by the C automatic_behavior() sequence.
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
# Camera / accelerometer setup
# -----------------------------
def initialize_optional_sensors():
    global accel, camera

    accel = safe_get_device("accel")
    if accel is not None:
        accel.enable(timestep)
        print("accel found")
    else:
        print("accel not found")

    camera = safe_get_device("camera")
    if camera is None:
        print("cam not found")
        return

    print("cam found")
    camera.enable(timestep)

    if hasattr(camera, "recognitionEnable"):
        camera.recognitionEnable(timestep)
    else:
        print("Warning: this camera object does not expose recognitionEnable().")

    print(f"cam size before step: {camera.getWidth()} x {camera.getHeight()}")

    # Match the C code's warmup steps so image/recognition data is available.
    for _ in range(3):
        step_robot()

    image = camera.getImage()
    if image:
        print("camera image OK after steps")
        try:
            width = camera.getWidth()
            r = camera.imageGetRed(image, width, 0, 0)
            g = camera.imageGetGreen(image, width, 0, 0)
            b = camera.imageGetBlue(image, width, 0, 0)
            print(f"top-left pixel = {r} {g} {b}")
        except Exception:
            # Different Webots Python versions expose image pixel helpers differently.
            pass
    else:
        print("camera image still null after steps")

# -----------------------------
# Camera recognition helpers
# -----------------------------
def get_object_model(obj):
    if hasattr(obj, "getModel"):
        return obj.getModel()
    return getattr(obj, "model", None)


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
        print(f"Could not read camera recognition objects: {exc}")
        return []


def find_box(objects, target_model="wooden box"):
    for obj in objects:
        model = get_object_model(obj)
        if model == target_model:
            return obj
    return None


def print_recognition_debug(objects):
    print(f"Detected objects: {len(objects)}")

    for i, obj in enumerate(objects):
        pos_img = get_object_position_on_image(obj)
        size_img = get_object_size_on_image(obj)
        pos = get_object_position(obj)

        print(f"Object {i}:")
        print(f"  id: {get_object_id(obj)}")
        print(f"  model: {get_object_model(obj)}")
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


def print_camera_size_debug():
    if camera is None:
        return

    print(f"Camera size: {camera.getWidth()} x {camera.getHeight()}")

# -----------------------------
# Behaviors added from the C code
# -----------------------------
def grab_box_sequence():
    """Grip the box, then lift it in stages instead of snapping to reset."""
    base_reset()

    # Re-apply holding strength before the loaded move. This helps if Webots
    # resets motor effort after a world reload or if a device was reconfigured.
    for motor in arm_motors:
        motor.setVelocity(ARM_MOTOR_VELOCITY)
        try_set_motor_strength(motor, ARM_MOTOR_HOLD_TORQUE)
    try_set_motor_strength(gripper, GRIPPER_HOLD_FORCE)

    gripper_release()
    passive_wait(0.5)

    arm_set_height(ARM_FRONT_FLOOR)
    passive_wait(6.0)

    gripper_grip()
    passive_wait(5.0)

    # Important: do not jump directly from floor to reset while loaded.
    # Lift to a safer front carrying pose first, then reset.
    arm_set_height(ARM_FRONT_PLATE)
    passive_wait(2.5)

    arm_set_height(ARM_RESET)
    passive_wait(2.5)

    base_reset()


def automatic_behavior():
    passive_wait(2.0)
    gripper_release()
    arm_set_height(ARM_FRONT_CARDBOARD_BOX)
    passive_wait(4.0)
    gripper_grip()
    passive_wait(1.0)
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


def process_box_pickup_state_machine(objects):
    """Python version of the C MODE_SEARCH/MODE_CENTER/MODE_APPROACH/MODE_GRAB logic."""
    global mode

    if camera is None:
        return

    box = find_box(objects, "wooden box")
    image_center_x = camera.getWidth() // 2

    if mode == MODE_DONE:
        base_reset()
        return

    if box is None:
        mode = MODE_SEARCH
        print("No box detected")
        base_turn_left_increment()
        return

    pos_img = get_object_position_on_image(box)
    pos = get_object_position(box)

    x = int(pos_img[0])
    error_x = x - image_center_x

    # Same relative-position fields used by the C code.
    rel_x = pos[0]
    rel_z = pos[2]

    print(
        f"Box img_x={x} error_x={error_x} "
        f"rel=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
    )

    if mode == MODE_SEARCH:
        mode = MODE_CENTER

    elif mode == MODE_CENTER:
        if abs(error_x) > 5:
            if error_x < 0:
                base_strafe_left_increment()
            else:
                base_strafe_right_increment()
        else:
            base_reset()
            mode = MODE_APPROACH

    elif mode == MODE_APPROACH:
        if abs(error_x) > 25:
            print("Recentering")
            mode = MODE_CENTER
        elif rel_x > 0.295:
            # This preserves the C code's use of rel_x as the approach threshold.
            # If your robot moves the wrong way, check whether Webots reports the
            # forward distance in position[0] or position[2] for your camera frame.
            print(f"Approaching box, rel_x={rel_x:.3f}, rel_z={rel_z:.3f}")
            base_forwards_increment()
        else:
            base_reset()
            mode = MODE_GRAB

    elif mode == MODE_GRAB:
        grab_box_sequence()
        mode = MODE_DONE

# -----------------------------
# Keyboard handling
# -----------------------------
def handle_keyboard(key, previous_key):
    global box_pickup_enabled, mode, autonomous_mode

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
        print("Reset")
        base_reset()
        arm_reset()
        mode = MODE_SEARCH

    elif key in (ord("+"), 388, 65585):
        print("Grip")
        gripper_grip()

    elif key in (ord("-"), 390):
        print("Ungrip")
        gripper_release()

    elif key in (Keyboard.SHIFT + Keyboard.UP, 332):
        print("Increase arm height")
        arm_increase_height()

    elif key in (Keyboard.SHIFT + Keyboard.DOWN, 326):
        print("Decrease arm height")
        arm_decrease_height()

    elif key in (Keyboard.SHIFT + Keyboard.RIGHT, 330):
        print("Increase arm orientation")
        arm_increase_orientation()

    elif key in (Keyboard.SHIFT + Keyboard.LEFT, 328):
        print("Decrease arm orientation")
        arm_decrease_orientation()

    elif key in (ord("b"), ord("B")):
        box_pickup_enabled = not box_pickup_enabled
        print(f"Automatic box pickup: {'ON' if box_pickup_enabled else 'OFF'}")
        if box_pickup_enabled and mode == MODE_DONE:
            mode = MODE_SEARCH

    elif key in (ord("d"), ord("D")):
        print("Running scripted demo behavior")
        automatic_behavior()

    elif key in (ord("a"), ord("A")):
        # Kept from the original Python helper text, but guarded because the
        # Theta* autonomous_step() function in the provided Python file was commented out.
        autonomous_mode = not autonomous_mode
        print(f"Theta* autonomous placeholder: {'ON' if autonomous_mode else 'OFF'}")

    else:
        print("Wrong keyboard input")

# -----------------------------
# Initial Reset State
# -----------------------------
base_reset()
arm_reset()
gripper_release()

initialize_optional_sensors()

# Match the C code's initial wait.
passive_wait(2.0)
passive_wait(2.0)

# Match the C code's optional demo argument behavior.
if len(sys.argv) > 1 and sys.argv[1] == "demo":
    automatic_behavior()

display_helper_message()

# -----------------------------
# Main loop
# -----------------------------
previous_key = -1
autonomous_warning_printed = False

while robot.step(timestep) != -1:
    if autonomous_mode:
        # The provided Python file had autonomous_step() commented out inside a
        # triple-quoted block, so do not call it unless the user later restores it.
        if "autonomous_step" in globals():
            autonomous_step()
            previous_key = keyboard.getKey()
            continue
        elif not autonomous_warning_printed:
            print("autonomous_step() is not defined; leaving normal control active.")
            autonomous_warning_printed = True

    objects = get_recognition_objects()

    if box_pickup_enabled:
        process_box_pickup_state_machine(objects)

    if DEBUG_SENSOR_PRINTS:
        print_recognition_debug(objects)
        print_accelerometer_debug()
        print_camera_size_debug()

    key = keyboard.getKey()
    handle_keyboard(key, previous_key)
    previous_key = key
