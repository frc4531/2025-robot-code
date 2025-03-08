import math

import commands2
import wpilib
import wpimath

from constants.swerve_constants import OIConstants
from subsystems.drive_subsystem import DriveSubsystem


class InputDrive(commands2.Command):

    def __init__(self, drive_sub: DriveSubsystem, x_input, y_input, z_input) -> None:
        super().__init__()

        self.x_input = x_input
        self.y_input = y_input
        self.z_input = z_input

        self.driver_controller = wpilib.Joystick(0)
        self.drive_sub = drive_sub
        self.addRequirements(self.drive_sub)

    def execute(self) -> None:
        forward = self.driver_controller.getY()
        strafe = self.driver_controller.getX()

        gyro_degrees = self.drive_sub.get_heading()
        gyro_radians = gyro_degrees * math.pi/180
        temp = forward * math.cos(gyro_radians) + strafe * math.sin(gyro_radians)
        strafe = -forward * math.sin(gyro_radians) + strafe * math.cos(gyro_radians)
        fwd = temp

        self.drive_sub.drive(
            -self.x_input,
            -self.y_input,
            -self.z_input,
            False,
            False,
        )

        # ----- DRIVE CODE FROM 2024-robot-code -----

        # self.robotDrive.drive(
        #     -wpimath.applyDeadband(
        #         (self.driverController.getY() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))) +
        #         (self.driverController.getX() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))),
        #         OIConstants.kDriveDeadband
        #     ),
        #     -wpimath.applyDeadband(
        #         (-self.driverController.getY() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))) +
        #         (self.driverController.getX() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))),
        #         OIConstants.kDriveDeadband
        #     ),
        #     -wpimath.applyDeadband(
        #         self.driverController.getZ(), OIConstants.kDriveDeadband
        #     ),
        #     True,
        #     False,
        # )

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.drive_sub.drive(
            0,
            0,
            0,
            True,
            True,
        )
