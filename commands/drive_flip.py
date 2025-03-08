import math

import commands2
import wpilib
import wpimath.controller

from subsystems.drive_subsystem import DriveSubsystem
from constants.swerve_constants import OIConstants


class DriveFlip(commands2.Command):

    def __init__(self, drive_sub: DriveSubsystem, stick: wpilib.Joystick) -> None:
        super().__init__()

        self.drive_sub = drive_sub
        self.addRequirements(self.drive_sub)

        self.driver_controller = stick

        self.current_heading = 0
        self.target_heading = 0

        self.min_rot_speed = 0.05
        self.max_rot_speed = 0.8
        self.target_threshold = 5

        self.rot_controller = wpimath.controller.PIDController(0.025, 0, 0)

    def initialize(self):
        self.current_heading = self.drive_sub.get_heading()
        self.target_heading = self.current_heading + 180

        if self.target_heading >= 180:
            self.target_heading = self.target_heading - 360

    def execute(self) -> None:
        pid_output = self.rot_controller.calculate(self.drive_sub.get_heading(), self.target_heading)
        z_output = max(min(pid_output, self.max_rot_speed), -self.max_rot_speed)

        if 0 < z_output < self.min_rot_speed:
            z_output = self.min_rot_speed
        elif -self.min_rot_speed > z_output > 0:
            z_output = -self.min_rot_speed

        self.drive_sub.drive(
            -wpimath.applyDeadband(
                (self.driver_controller.getY() * math.sin(self.drive_sub.get_heading() * (math.pi / 180))) +
                (self.driver_controller.getX() * math.cos(self.drive_sub.get_heading() * (math.pi / 180))),
                OIConstants.kDriveDeadband
            ),
            -wpimath.applyDeadband(
                (-self.driver_controller.getY() * math.cos(self.drive_sub.get_heading() * (math.pi / 180))) +
                (self.driver_controller.getX() * math.sin(self.drive_sub.get_heading() * (math.pi / 180))),
                OIConstants.kDriveDeadband
            ),
            wpimath.applyDeadband(
                z_output, OIConstants.kDriveDeadband
            ),
            True,
            False,
        )

    def isFinished(self) -> bool:
        return self.target_heading - self.target_threshold < self.drive_sub.get_heading() < self.target_heading + self.target_threshold
