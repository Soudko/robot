from controller import Supervisor
import math

TIME_STEP = 32


ROOM_MIN_X = -1.5
ROOM_MAX_X = 1.5
ROOM_MIN_Y = -1.5
ROOM_MAX_Y = 1.5

WALL_MARGIN = 0.12

COVER_MIN_X = ROOM_MIN_X + WALL_MARGIN
COVER_MAX_X = ROOM_MAX_X - WALL_MARGIN
COVER_MIN_Y = ROOM_MIN_Y + WALL_MARGIN
COVER_MAX_Y = ROOM_MAX_Y - WALL_MARGIN

HOME_X = COVER_MIN_X
HOME_Y = COVER_MIN_Y
HOME_HEADING = 0.0  


MAX_WHEEL_SPEED = 6.0
FORWARD_SPEED = 4.0
SLOW_FORWARD_SPEED = 3.0
TURN_SPEED = 2.5

POSITION_TOLERANCE = 0.03
ANGLE_TOLERANCE = math.radians(3)

ROW_STEP = 0.10


DIRT_DEFS = [
    "DIRT_1", "DIRT_2", "DIRT_3", "DIRT_4", "DIRT_5",
    "DIRT_6", "DIRT_7", "DIRT_8", "DIRT_9"
]
DIRT_REACHED_TOLERANCE = 0.05

STATE_GO_TO_HOME_START = "GO_TO_HOME_START"
STATE_ALIGN_HOME = "ALIGN_HOME"

STATE_COVER_ROW = "COVER_ROW"
STATE_TURN_UP = "TURN_UP"
STATE_MOVE_UP = "MOVE_UP"
STATE_TURN_NEXT_ROW = "TURN_NEXT_ROW"

STATE_GO_TO_DIRT = "GO_TO_DIRT"
STATE_RETURN_TO_RESUME_POINT = "RETURN_TO_RESUME_POINT"
STATE_RETURN_TO_RESUME_HEADING = "RETURN_TO_RESUME_HEADING"

STATE_RETURN_HOME = "RETURN_HOME"
STATE_ALIGN_FINAL = "ALIGN_FINAL"
STATE_FINISHED = "FINISHED"

def clamp(value, low, high):
    return max(low, min(high, value))

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

robot = Supervisor()

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

self_node = robot.getSelf()
translation_field = self_node.getField("translation")
rotation_field = self_node.getField("rotation")

def get_robot_pose():
    pos = translation_field.getSFVec3f()
    x = pos[0]
    y = pos[1]

    rot = rotation_field.getSFRotation()
    axis_x, axis_y, axis_z, angle = rot

    if axis_z < 0:
        angle = -angle

    yaw = normalize_angle(angle)
    return x, y, yaw

def set_wheel_speeds(left_speed, right_speed):
    left_motor.setVelocity(clamp(left_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
    right_motor.setVelocity(clamp(right_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))

def stop_robot():
    set_wheel_speeds(0.0, 0.0)

def rotate_to_heading(target_heading):
    _, _, yaw = get_robot_pose()
    error = normalize_angle(target_heading - yaw)

    if abs(error) < ANGLE_TOLERANCE:
        stop_robot()
        return True

    turn = clamp(2.2 * error, -TURN_SPEED, TURN_SPEED)
    set_wheel_speeds(-turn, turn)
    return False

def drive_to_point(target_x, target_y, forward_speed=FORWARD_SPEED):
    x, y, yaw = get_robot_pose()

    dx = target_x - x
    dy = target_y - y
    dist = math.hypot(dx, dy)

    if dist < POSITION_TOLERANCE:
        stop_robot()
        return True

    target_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(target_heading - yaw)

    if abs(heading_error) > math.radians(18):
        turn = clamp(2.4 * heading_error, -TURN_SPEED, TURN_SPEED)
        set_wheel_speeds(-turn, turn)
        return False

    correction = clamp(3.0 * heading_error, -1.8, 1.8)
    left_speed = forward_speed - correction
    right_speed = forward_speed + correction
    set_wheel_speeds(left_speed, right_speed)
    return False

class Dirt:
    def __init__(self, def_name, node):
        self.def_name = def_name
        self.node = node
        self.discovered = False
        self.cleaned = False

    def get_position(self):
        if self.node is None:
            return None
        t = self.node.getField("translation").getSFVec3f()
        return t[0], t[1], t[2]

    def mark_cleaned(self):
        self.cleaned = True
        if self.node is not None:
            tf = self.node.getField("translation")
            pos = tf.getSFVec3f()
            tf.setSFVec3f([pos[0], pos[1], -1.0])

dirts = []
for def_name in DIRT_DEFS:
    node = robot.getFromDef(def_name)
    if node is not None:
        dirts.append(Dirt(def_name, node))

DETECTION_RADIUS = 0.25

def point_in_detection_circle(robot_x, robot_y, point_x, point_y):
    dx = point_x - robot_x
    dy = point_y - robot_y
    return math.hypot(dx, dy) <= DETECTION_RADIUS

def discover_visible_dirts():
    robot_x, robot_y, robot_yaw = get_robot_pose()
    newly_discovered = []

    for dirt in dirts:
        if dirt.cleaned:
            continue

        pos = dirt.get_position()
        if pos is None:
            continue

        dirt_x, dirt_y, _ = pos

        if point_in_detection_circle(robot_x, robot_y, dirt_x, dirt_y):
            if not dirt.discovered:
                dirt.discovered = True
                newly_discovered.append(dirt)

    return newly_discovered

def get_pending_discovered_dirt():
    for dirt in dirts:
        if dirt.discovered and not dirt.cleaned:
            return dirt
    return None

def all_cleaned():
    return all(d.cleaned for d in dirts)

rows = []
yy = COVER_MIN_Y
while yy <= COVER_MAX_Y + 1e-9:
    rows.append(min(yy, COVER_MAX_Y))
    yy += ROW_STEP

current_row_index = 0
moving_right = True

resume_x = None
resume_y = None
resume_heading = None
resume_state = None
current_target_dirt = None

state = STATE_GO_TO_HOME_START

print("=== Cleaning controller started ===")
print(f"Loaded dirts: {len(dirts)}")
print(f"Rows: {len(rows)}")

while robot.step(TIME_STEP) != -1:
    x, y, yaw = get_robot_pose()

    newly_discovered = discover_visible_dirts()
    if newly_discovered:
        print("Discovered:", [d.def_name for d in newly_discovered])

    if state == STATE_COVER_ROW and current_target_dirt is None:
        pending = get_pending_discovered_dirt()
        if pending is not None:
            resume_x = x
            resume_y = y
            resume_heading = yaw
            resume_state = STATE_COVER_ROW
            current_target_dirt = pending
            state = STATE_GO_TO_DIRT
            print(f"Interrupt -> go clean {pending.def_name}")

    if state == STATE_GO_TO_HOME_START:
        reached = drive_to_point(HOME_X, HOME_Y, forward_speed=SLOW_FORWARD_SPEED)
        if reached:
            stop_robot()
            state = STATE_ALIGN_HOME
            print("Reached home start")

    elif state == STATE_ALIGN_HOME:
        if rotate_to_heading(HOME_HEADING):
            stop_robot()
            state = STATE_COVER_ROW
            current_row_index = 0
            moving_right = True
            print("Aligned to start heading -> begin coverage")

    elif state == STATE_COVER_ROW:
        row_y = rows[current_row_index]
        target_x = COVER_MAX_X if moving_right else COVER_MIN_X

        reached = drive_to_point(target_x, row_y, forward_speed=FORWARD_SPEED)

        if reached or (
            moving_right and x >= COVER_MAX_X - 0.02
        ) or (
            (not moving_right) and x <= COVER_MIN_X + 0.02
        ):
            stop_robot()

            if current_row_index >= len(rows) - 1:
                state = STATE_RETURN_HOME
                print("Coverage complete -> return home")
            else:
                state = STATE_TURN_UP
                print(f"Finished row {current_row_index} -> turn up")

    elif state == STATE_TURN_UP:
        if rotate_to_heading(math.pi / 2):
            stop_robot()
            state = STATE_MOVE_UP
            print("Facing up")

    elif state == STATE_MOVE_UP:
        next_row_y = rows[current_row_index + 1]
        edge_x = COVER_MAX_X if moving_right else COVER_MIN_X

        reached = drive_to_point(edge_x, next_row_y, forward_speed=SLOW_FORWARD_SPEED)

        if reached or y >= next_row_y - 0.02:
            stop_robot()
            current_row_index += 1
            state = STATE_TURN_NEXT_ROW
            print(f"Reached row {current_row_index}")

    elif state == STATE_TURN_NEXT_ROW:
        target_heading = math.pi if moving_right else 0.0
        if rotate_to_heading(target_heading):
            stop_robot()
            moving_right = not moving_right
            state = STATE_COVER_ROW
            print("Start next row")

    elif state == STATE_GO_TO_DIRT:
        if current_target_dirt is None or current_target_dirt.cleaned:
            state = STATE_RETURN_TO_RESUME_POINT
        else:
            pos = current_target_dirt.get_position()
            if pos is None:
                current_target_dirt.mark_cleaned()
                state = STATE_RETURN_TO_RESUME_POINT
            else:
                dirt_x, dirt_y, _ = pos
                reached = drive_to_point(dirt_x, dirt_y, forward_speed=SLOW_FORWARD_SPEED)

                if reached or distance(x, y, dirt_x, dirt_y) < DIRT_REACHED_TOLERANCE:
                    stop_robot()
                    print(f"Cleaned {current_target_dirt.def_name}")
                    current_target_dirt.mark_cleaned()
                    state = STATE_RETURN_TO_RESUME_POINT

    elif state == STATE_RETURN_TO_RESUME_POINT:
        reached = drive_to_point(resume_x, resume_y, forward_speed=SLOW_FORWARD_SPEED)
        if reached:
            stop_robot()
            state = STATE_RETURN_TO_RESUME_HEADING
            print("Returned to resume point")

    elif state == STATE_RETURN_TO_RESUME_HEADING:
        if rotate_to_heading(resume_heading):
            stop_robot()
            current_target_dirt = None
            state = resume_state
            print("Resume heading restored -> continue coverage")

    elif state == STATE_RETURN_HOME:
        reached = drive_to_point(HOME_X, HOME_Y, forward_speed=SLOW_FORWARD_SPEED)
        if reached:
            stop_robot()
            state = STATE_ALIGN_FINAL
            print("Reached home position")

    elif state == STATE_ALIGN_FINAL:
        if rotate_to_heading(HOME_HEADING):
            stop_robot()
            state = STATE_FINISHED
            print("Mission completed")

    elif state == STATE_FINISHED:
        stop_robot()

