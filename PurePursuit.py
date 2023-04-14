import numpy as np


class Pure_Pursuit:
    def __init__(self, look_ahead_distance):
        """
        Initialize the Pure Pursuit controller.

        :param look_ahead_distance: The lookahead distance used for determining the goal point.
        """
        self.look_ahead_distance = look_ahead_distance

    def find_goal_point(self, path, robot_position):
        """
        Find the goal point on the path based on the lookahead distance.

        :param path: The path as an array of (x, y) points.
        :param robot_position: The current position of the robot as a tuple (x, y).

        :return: The goal point as a tuple (x, y).
        """
        try:
            if not path:
                raise ValueError("Path is empty.")

            min_distance = float("inf")
            goal_point = path[0]

            for point in path:
                distance = np.linalg.norm(
                    np.array(robot_position) - np.array(point))

                if distance < min_distance and distance > self.look_ahead_distance:
                    min_distance = distance
                    goal_point = point

            return goal_point

        except Exception as e:
            print(f"Error in finding goal point: {e}")
            return None

    def calculate_steering_angle(self, robot_position, robot_orientation, goal_point):
        """
        Calculate the steering angle based on the robot's position, orientation, and goal point.

        :param robot_position: The current position of the robot as a tuple (x, y).
        :param robot_orientation: The current orientation of the robot in radians.
        :param goal_point: The goal point as a tuple (x, y).

        :return: The steering angle in radians.
        """
        try:
            if goal_point is None:
                raise ValueError("Goal point is None.")

            robot_to_goal_vector = np.array(
                goal_point) - np.array(robot_position)
            robot_orientation_vector = np.array(
                [np.cos(robot_orientation), np.sin(robot_orientation)])

            angle = np.arctan2(robot_to_goal_vector[1], robot_to_goal_vector[0]) - np.arctan2(
                robot_orientation_vector[1], robot_orientation_vector[0])

            # Normalize the angle to be within the range of -pi to pi
            angle = np.arctan2(np.sin(angle), np.cos(angle))

            return angle

        except Exception as e:
            print(f"Error in calculating steering angle: {e}")
            return None

    def control(self, path, robot_position, robot_orientation):
        """
        Calculate the control signal (steering angle) based on the path and robot's state.

        :param path: The path as an array of (x, y) points.
        :param robot_position: The current position of the robot as a tuple (x, y).
        :param robot_orientation: The current orientation of the robot in radians.

        :return: The control signal as the steering angle in radians.
        """
        goal_point = self.find_goal_point(path, robot_position)
        steering_angle = self.calculate_steering_angle(
            robot_position, robot_orientation, goal_point)

        return steering_angle
