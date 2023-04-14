import numpy as np


class Zonal_Defense:

    # Constants
    MARGIN_WIDTH = 600
    zones = []

    def __init__(self, field_width, field_height):
        """
        Initialize the Zonal_Defense class.

        :param field_width: The width of the field.
        :param field_height: The height of the field.
        """

        # Divide the field width by the number of defending robots
        zone_width = field_width / 3
        # Covering only one side of the field (our team's side)
        zone_length = field_height / 2.5

        # Define zones based on the field division
        self.zones = [
            ((-zone_length, (zone_width / 2) - self.MARGIN_WIDTH),
             (0, zone_length)),  # Zone 0
            ((-zone_length, -zone_width / 2),
             (0, zone_width)),  # Zone 1
            ((-zone_length, -zone_length),
             (0, (-zone_width / 2) + self.MARGIN_WIDTH))  # Zone 2
        ]

    def get_zone_id(self, position):
        """
        Get the zone ID of a given robot_position.

        :param robot_position: The position of the robot.
        :return: The zone ID if the robot is inside a zone, or None if the robot is outside the zones.
        """
        x, y = position
        for i, zone in enumerate(self.zones):
            (x_min, y_min), (x_max, y_max) = zone
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return i
        return None

    def get_zone_position(self, zone):
        """
        Move the robot to the center of the specified zone.

        :param zone_id: The zone ID to move the robot to.
        """
        (x_min, y_min), (x_max, y_max) = zone
        target_position = np.array(
            [x_min + x_max / 2, (y_min + y_max - self.MARGIN_WIDTH) / 2])

        return target_position

    def get_robot_zone(self, robot):
        robot_zone = 0
        if (robot.robot_id == 0):
            robot_zone = 1
        elif (robot.robot_id == 1):
            robot_zone = 0
        elif (robot.robot_id == 2):
            robot_zone = 2

        return self.zones[robot_zone]
