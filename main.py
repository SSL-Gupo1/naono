#!/usr/bin/env python3
import argparse
import rospy
import math
import numpy as np
import re
import py_trees
import py_trees_ros
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from krssg_ssl_msgs.msg import SSL_DetectionFrame, SSL_DetectionRobot
from grsim_ros_bridge_msgs.msg import SSL

from PurePursuit import Pure_Pursuit
from ZonalDefense import Zonal_Defense

# Constants
FIELD_LENGTH = 4000
FIELD_WIDTH = 4000
KP = 0.004
KICK_DISTANCE = 40
DISTANCE_TO_BALL = 80
KICK_SPEED = 0.2
MAX_LINEAR_VELOCITY = 0.2
MAX_ANGULAR_VELOCITY = 0.3
ANGLE_TOLERANCE = 0.3  # In radians, adjust as needed
GOAL_TOP_Y_MOVEMENT = 430
DELTA_X = 150

GOALKEEPER_ID = 4
DEFENDER_ID = 0
MIDFILEDER_1_ID = 1
MIDFILEDER_2_ID = 2
STRIKER_ID = 3
TEAM_COLOR = "blue"

# Global variables
ball_pose = Pose()
robot_publishers = []
robots = [SSL_DetectionRobot() for _ in range(5)]
robots_yellow = [SSL_DetectionRobot() for _ in range(5)]

ball_position = (0, 0)
goal_position = (2000 + DELTA_X, 0)
goal_width = 1000
goal_y_min = goal_position[1] - goal_width / 2
goal_y_max = goal_position[1] + goal_width / 2
is_ball_in_my_field = False
ball_posession = False


class BallPossessionCondition(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        closest_robots = get_ordered_closest_robots()
        ball_posession = TEAM_COLOR in closest_robots[0]

        if ball_posession:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class DefendAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        move_goalkeeper()
        robot = robots[STRIKER_ID]
        rotate_then_move(robot, ball_position)

        zonalDefense = Zonal_Defense(FIELD_WIDTH, FIELD_LENGTH)
        filtered_robots = [
            robot for robot in robots if robot.robot_id in (0, 1, 2)]

        for robot in filtered_robots:
            current_zone = zonalDefense.get_zone_id((robot.x, robot.y))
            if current_zone == None or current_zone != robot.robot_id:
                robot_zone = zonalDefense.get_robot_zone(robot)
                target_position = zonalDefense.get_zone_position(robot_zone)
                rotate_then_move(robot, target_position)
            else:
                rotate_then_move(robot, ball_position)

        if is_goal_scored():
            rospy.loginfo("GOOOOLL!!!!")
            move_all_initial_position()
            # TODO: set robots original position
        elif (is_ball_outside_field()):
            rospy.loginfo("FUERA")
            move_all_initial_position()
            # TODO: set robots original position

        return py_trees.common.Status.SUCCESS


class AttackAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        mine_robots_ordered_by_ball_proximity = get_mine_robots()

        ball_possession_robot = mine_robots_ordered_by_ball_proximity[0]
        if ball_possession_robot is not None:
            # Move the robot with ball possession towards the opponent's goal
            move_attacker(ball_possession_robot)

            # Assign roles to the other robots
            support_robot = mine_robots_ordered_by_ball_proximity[1]
            pass_receiver_robot = mine_robots_ordered_by_ball_proximity[2]
            defensive_robot = mine_robots_ordered_by_ball_proximity[3]

            # Move the support robot to a supporting position
            move_to_support_position(support_robot, ball_possession_robot)

            # Move the pass receiver robot to an open position for receiving a pass
            move_to_pass_receiving_position(
                pass_receiver_robot, ball_possession_robot)

        # defense tactic
        move_to_defensive_position(defensive_robot)

        if is_goal_scored():
            rospy.loginfo("GOOOOLL!!!!")
            move_all_initial_position()
            # TODO: set robots original position
        elif (is_ball_outside_field()):
            rospy.loginfo("FUERA")
            move_all_initial_position()
            # TODO: set robots original position

        return py_trees.common.Status.SUCCESS


# Callback function to update the position of the ball
def position_ball(data):
    global ball_position
    try:
        if data.balls:
            ball_position = (data.balls[0].x, data.balls[0].y)
    except Exception as e:
        rospy.logerr("Error updating ball position: %s", str(e))


# Callback function to update the position of the blue robots
def update_blue_robots(data):
    global robots

    try:
        for robot in data.robots_blue:
            if robot.robot_id < len(robots):
                robots[robot.robot_id] = robot
    except Exception as e:
        rospy.logerr("Error in processing blue robots data: %s", str(e))


# Callback function to update the position of the yellow robots
def update_yellow_robots(data):
    global robots_yellow

    try:
        for robot in data.robots_yellow:
            if robot.robot_id < len(robots):
                robots_yellow[robot.robot_id] = robot
    except Exception as e:
        rospy.logerr("Error in processing yellow robots data: %s", str(e))


def set_yellow_goal_posittion():
    global goal_position
    goal_position = (-(FIELD_LENGTH/2) - DELTA_X, 0)


def limit_position_within_field(x, y):
    # Define field boundaries
    min_x = -FIELD_LENGTH
    max_x = FIELD_LENGTH
    min_y = -FIELD_WIDTH
    max_y = FIELD_WIDTH

    # Limit the position within the field boundaries
    limited_x = max(min_x, min(x, max_x))
    limited_y = max(min_y, min(y, max_y))

    return limited_x, limited_y


def move_to_pass_receiving_position(pass_receiver_robot, ball_possession_robot):
    # Define the offset distance from the ball carrier
    offset_distance = 200  # Meters

    # Calculate the angle between the ball carrier and the opponent's goal
    goal_x, goal_y = goal_position

    # Calculate the target position for the pass receiver robot
    target_x = goal_x - offset_distance
    target_y = ball_possession_robot.y - offset_distance

    # Limit the target position within the field dimensions
    target_x, target_y = limit_position_within_field(target_x, target_y)

    # Move the pass receiver robot to the target position
    target_position = (target_x, target_y)
    rotate_then_move(pass_receiver_robot, target_position)


def move_to_support_position(support_robot, ball_possession_robot):
    # Calculate support position coordinates
    support_distance = 500  # Meters, adjust as needed
    goal_x, goal_y = goal_position

    dx = goal_x - ball_possession_robot.x
    dy = goal_y - ball_possession_robot.y

    angle_to_goal = math.atan2(dy, dx)

    support_x = ball_possession_robot.x - \
        support_distance * math.cos(angle_to_goal)
    support_y = ball_possession_robot.y - \
        support_distance * math.sin(angle_to_goal)

    # Limit the support position within the field
    limited_x, limited_y = limit_position_within_field(support_x, support_y)

    # Move the support robot to the support position
    target_position = (limited_x, limited_y)
    rotate_then_move(support_robot, target_position)


def move_to_defensive_position(defensive_robot):

    # Move the support robot to the defensive position
    target_position = (0, 0)
    rotate_then_move(defensive_robot, target_position)


def angle_difference(angle1, angle2):
    return math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2))


def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))


def angle_between_points(point1, point2):
    point1_np = np.array(point1)
    point2_np = np.array(point2)
    delta = point2_np - point1_np
    angle = np.arctan2(delta[1], delta[0])
    return angle


# Function to calculate the distance between robots and the ball
def get_closest_robot_index():
    min_distance = float("inf")
    closest_robot_index = -1

    for idx, robot in enumerate(robots):
        distance = euclidean_distance(ball_position, (robot.x, robot.y))

        if distance < min_distance and idx != GOALKEEPER_ID:
            min_distance = distance
            closest_robot_index = idx

    return closest_robot_index


def get_ordered_closest_robots():
    distances = []

    for idx, robot in enumerate(robots):
        distance = euclidean_distance(ball_position, (robot.x, robot.y))
        distances.append((f"blue_{idx}", distance))

    for idx, robot in enumerate(robots_yellow):
        distance = euclidean_distance(ball_position, (robot.x, robot.y))
        distances.append((f"yellow_{idx}", distance))

    # Sort the list by distance
    sorted_distances = sorted(distances, key=lambda x: x[1])

    # Extract the indices of the robots in the sorted order
    ordered_robot_indices = [item[0] for item in sorted_distances]

    return ordered_robot_indices


def get_mine_robots():
    distances = []

    for robot in robots:
        if (robot == robots[GOALKEEPER_ID]):
            continue
        distance = euclidean_distance(ball_position, (robot.x, robot.y))
        distances.append((robot, distance))

    # Sort the list by distance
    sorted_distances = sorted(distances, key=lambda x: x[1])

    # Extract the robot objects in the sorted order
    ordered_robots = [item[0] for item in sorted_distances]

    return ordered_robots


def get_number_from_string(s):
    match = re.search(r'\d+', s)
    return int(match.group(0)) if match else None

# Function to move the closest robot to the ball


def move_closest_robot_to_ball():
    closest_robot_index = get_closest_robot_index()
    move_robot_to_ball(robots[closest_robot_index])


def move_robot_to_ball(robot):

    try:
        # Calculate the angle between the robot and the ball
        target = calculate_target_position_before_ball()
        rotate_then_move(robot, target)

    # Handle any exceptions that might occur
    except Exception as e:
        rospy.logerr("Error moving robot: %s", str(e))


def is_robot_aligned_with_goal(robot):
    # Calculate the angle between the robot and the goal
    dx = goal_position[0] - robot.x
    dy = goal_position[1] - robot.y
    goal_angle = math.atan2(dy, dx)

    # Calculate the difference between the robot's orientation and the goal angle
    angle_diff = angle_difference(goal_angle, robot.orientation)

    # Convert the angle tolerance to radians
    angle_tolerance_radians = math.radians(ANGLE_TOLERANCE)

    # Check if the robot is aligned with the goal within the angle tolerance
    return abs(angle_diff) <= angle_tolerance_radians


def calculate_target_position_before_ball():

    slope = (goal_position[1] - ball_position[1]) / \
        (goal_position[0] - ball_position[0])

    target_x = ball_position[0] - DISTANCE_TO_BALL

    target_y = slope * (target_x - ball_position[0]) + ball_position[1]
    target_position = (target_x, target_y)

    return target_position


def kick(robot):
    ssl_msg = SSL()
    ssl_msg.cmd_vel.linear.x = 0
    ssl_msg.cmd_vel.linear.y = 0
    ssl_msg.cmd_vel.angular.z = 0
    ssl_msg.kicker = True  # Enable the kicker
    # ssl_msg.cmd_vel.linear.x = MAX_LINEAR_VELOCITY

    robot_publishers[robot.robot_id].publish(ssl_msg)


def rotate_then_move(robot, target_position):
    try:
        ssl_msg = SSL()

        distance = euclidean_distance(target_position, (robot.x, robot.y))

        algorithmPP = Pure_Pursuit(distance)
        angle_correction = algorithmPP.control(
            [target_position], (robot.x, robot.y), robot.orientation)

        # If the robot is facing the ball, move towards it
        if abs(angle_correction) < ANGLE_TOLERANCE:
            ssl_msg.cmd_vel.angular.z = 0
            ssl_msg.cmd_vel.linear.x = 0.2
        else:
            ssl_msg.cmd_vel.linear.x = 0
            # Rotate the robot towards the ball
            # as the angle correction is normalized i need the same sign
            ssl_msg.cmd_vel.angular.z = MAX_ANGULAR_VELOCITY * angle_correction

        # Publish the movement command for the robot
        robot_publishers[robot.robot_id].publish(ssl_msg)

    except ValueError as ve:
        rospy.logerr(f"ValueError in rotate_then_move: {ve}")
    except rospy.ROSException as re:
        rospy.logerr(f"ROSException in rotate_then_move: {re}")
    except Exception as e:
        rospy.logerr(f"Unexpected error in rotate_then_move: {e}")


def move_with_ball(robot):
    ssl_msg = SSL()

    ssl_msg.cmd_vel.linear.x = MAX_LINEAR_VELOCITY
    # Publish the movement command for the robot
    robot_publishers[robot.robot_id].publish(ssl_msg)


def move_goalkeeper():
    global ball_position, robots, goal_y_max, goal_y_min

    goalkeeper = robots[GOALKEEPER_ID]
    ssl_msg = SSL()

    # si la pelota esta en mi cancha muevo el arquero
    if (ball_position[0] <= 0):

        # Set the goalkeeper's target y-position equal to the ball's y-position limited by the goal width
        target_y = max(-GOAL_TOP_Y_MOVEMENT,
                       min(GOAL_TOP_Y_MOVEMENT, ball_position[1]))

        # Calculate the error in the y-axis between the goalkeeper and the ball
        error_y = target_y - goalkeeper.y

        # Set the goalkeeper's velocity in the y-axis based on the error
        ssl_msg.cmd_vel.linear.y = KP * error_y  # kp is the proportional gain

        # Set the goalkeeper's x-axis velocity to zero (to keep it on the goal line)
        ssl_msg.cmd_vel.linear.x = 0
        ssl_msg.cmd_vel.angular.z = 0

        # Publish the movement command for the goalkeeper
        robot_publishers[GOALKEEPER_ID].publish(ssl_msg)


def move_goalkeeper_initial_position():
    robot = robots[GOALKEEPER_ID]

    target_x = -1800
    target_y = 0

    # Calculate the error in the y-axis between the goalkeeper and the ball
    target = (target_x, target_y)

    rotate_then_move(robot, target)


def move_defender_initial_position():

    robot = robots[DEFENDER_ID]

    target_x = -1000
    target_y = 0

    # Calculate the error in the y-axis between the goalkeeper and the ball
    target = (target_x, target_y)

    rotate_then_move(robot, target)


def move_midfielder_1_initial_position():

    robot = robots[MIDFILEDER_1_ID]

    target_x = -800
    target_y = -500

    # Calculate the error in the y-axis between the goalkeeper and the ball
    target = (target_x, target_y)

    rotate_then_move(robot, target)


def move_midfielder_2_initial_position():

    robot = robots[MIDFILEDER_2_ID]

    target_x = -800
    target_y = 500

    # Calculate the error in the y-axis between the goalkeeper and the ball
    target = (target_x, target_y)

    rotate_then_move(robot, target)


def move_striker_initial_position():

    robot = robots[STRIKER_ID]

    target_x = -600
    target_y = 0

    # Calculate the error in the y-axis between the goalkeeper and the ball
    target = (target_x, target_y)

    rotate_then_move(robot, target)


def move_attacker(robot):

    # closest_robot_id = get_closest_robot_index()
    # if (robot.robot_id == closest_robot_id):
    # if (is_ball_in_my_field == False):
    if is_robot_aligned_with_goal(robot):
        # Calculate the distance between the robot and the ball
        robot_ball_dist = euclidean_distance(
            ball_position, (robot.x, robot.y))
        # If the robot is close enough to the ball, kick it
        if robot_ball_dist <= KICK_DISTANCE:
            rospy.loginfo("kick")
            kick(robot)
        # if robot is close enough to move with the ball
        else:
            rospy.loginfo("llevala atada")
            move_with_ball(robot)
    # If the robot is not close enough to the ball, move towards it
    else:
        move_robot_to_ball(robot)


def move_striker():

    # if (is_ball_in_my_field == False):
    robot = robots[STRIKER_ID]

    move_attacker(robot)


def move_all_initial_position():
    # move_goalkeeper_initial_position()
    move_defender_initial_position()
    move_midfielder_1_initial_position()
    move_midfielder_2_initial_position()
    move_striker_initial_position()


def is_ball_outside_field():
    global ball_position

    half_length = FIELD_LENGTH / 2
    half_width = FIELD_WIDTH / 2

    if ball_position[0] < -half_length or ball_position[0] > half_length:
        return True

    if ball_position[1] < -half_width or ball_position[1] > half_width:
        return True

    return False


def is_goal_scored():
    """
    Checks if a goal has been scored based on the ball's position and the goal area.

    :param ball_position: A tuple containing the (x, y) coordinates of the ball.
    :param goal_position: A tuple containing the (x, y) coordinates of the center of the goal.
    :param goal_width: The width of the goal area along the y-axis.
    :return: True if a goal has been scored, False otherwise.
    """
    global goal_y_max, goal_y_min

    # Check if the ball's x-coordinate is beyond the goal line and if its y-coordinate is within the goal area
    if (ball_position[0] > goal_position[0] - DELTA_X) and goal_y_min <= ball_position[1] <= goal_y_max:
        return True
    # if ball_position[0] > (FIELD_LENGTH / 2) and goal_y_min <= ball_position[1] <= goal_y_max:
    #     return True  # gol blue
    # elif ball_position[0] < (-FIELD_LENGTH / 2) and goal_y_min <= ball_position[1] <= goal_y_max:
    #     return True  # gol yellow

    return False


def stop_all_robots():
    """
    Stops all robots by sending zero velocity commands.

    :param pubs: A list of publishers for each robot.
    """
    ssl_msg = SSL()
    ssl_msg.cmd_vel.linear.x = 0
    ssl_msg.cmd_vel.linear.y = 0
    ssl_msg.cmd_vel.angular.z = 0

    for robot in robots:
        robot.orientation = 0

    for pub in robot_publishers:
        pub.publish(ssl_msg)


def main():
    global robot_publishers, goal_position, is_ball_in_my_field, ball_posession
    print(f"Your team color is {TEAM_COLOR}")

    rospy.init_node("nanode", anonymous=False)

    rospy.Subscriber("/vision", SSL_DetectionFrame, position_ball)
    rospy.Subscriber("/vision", SSL_DetectionFrame, update_blue_robots)
    rospy.Subscriber("/vision", SSL_DetectionFrame, update_yellow_robots)

    robot_publishers = [
        rospy.Publisher(f'/robot_{TEAM_COLOR}_{i}/cmd', SSL, queue_size=10) for i in range(len(robots))
    ]

    rate = rospy.Rate(10)

    # Behavior tree implementation

    # Create behavior tree nodes
    ball_possession_condition = BallPossessionCondition("Ball Possession")
    defend_action = DefendAction("Defend")
    attack_action = AttackAction("Attack")

    # Assemble behavior tree
    root = py_trees.composites.Selector("Root")

    attack_sequence = py_trees.composites.Sequence("Attack Sequence")
    attack_sequence.add_child(ball_possession_condition)
    attack_sequence.add_child(attack_action)

    defense_sequence = py_trees.composites.Sequence("Defense Sequence")
    defense_sequence.add_child(
        py_trees.decorators.Inverter(child=ball_possession_condition))
    defense_sequence.add_child(defend_action)

    root.add_child(attack_sequence)
    root.add_child(defense_sequence)

    # Initialize and run behavior tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(timeout=15)
    tree.tick_tock(500)  # Specify the desired tick period in ms

    while not rospy.is_shutdown():

        if (TEAM_COLOR == "yellow"):
            set_yellow_goal_posittion()
            is_ball_in_my_field = ball_position[0] > 0
        else:
            is_ball_in_my_field = ball_position[0] < 0


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Set the team color for RoboCup SSL')
    parser.add_argument('--color', type=str, required=True, choices=['blue', 'yellow'],
                        help='The team color (either "blue" or "yellow")')

    args = parser.parse_args()
    TEAM_COLOR = args.color
    main()
