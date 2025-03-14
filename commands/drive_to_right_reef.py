import math

import commands2
import ntcore
import wpilib
import wpimath.controller

from constants.swerve_constants import OIConstants
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.vision_subsystem import VisionSubsystem


class DriveToRightReef(commands2.Command):

    def __init__(self, drive_sub: DriveSubsystem, vision_sub: VisionSubsystem, stick: wpilib.Joystick) -> None:
        super().__init__()

        self.drive_sub = drive_sub
        self.vision_sub = vision_sub
        self.addRequirements(self.drive_sub, self.vision_sub)

        self.driver_controller = stick

        # Constants
        self.max_strafe_speed = 0.6
        self.min_strafe_speed = 0.05

        self.max_forward_speed = 0.6
        self.min_forward_speed = 0.05

        # Y Speed Controller
        self.strafe_controller = wpimath.controller.PIDController(1, 0, 0)
        self.strafe_controller.setSetpoint(-21)
        # X Speed Controller
        self.forward_controller = wpimath.controller.PIDController(1, 0, 0)
        self.forward_controller.setSetpoint(-0.39)

        # network tables
        nt_instance = ntcore.NetworkTableInstance.getDefault()
        right_reef_table = nt_instance.getTable("right_reef_table")

        self.strafe_entry = right_reef_table.getDoubleTopic("strafe_entry").publish()
        self.forward_entry = right_reef_table.getDoubleTopic("forward_entry").publish()

    def execute(self) -> None:
        if self.vision_sub.front_v_entry == 1:
            pid_strafe_output = self.strafe_controller.calculate(self.vision_sub.front_y_entry)
            pid_forward_output = self.forward_controller.calculate(self.vision_sub.front_x_entry)

            y_output = max(min(pid_strafe_output, self.max_strafe_speed), -self.max_strafe_speed)
            x_output = max(min(pid_forward_output, self.max_forward_speed), -self.max_forward_speed)

            if 0 < y_output < self.min_strafe_speed:
                y_output = self.min_strafe_speed
            elif -self.min_strafe_speed > y_output > 0:
                y_output = -self.min_strafe_speed

            if 0 < x_output < self.min_forward_speed:
                x_output = self.min_forward_speed
            elif -self.min_forward_speed > x_output > 0:
                x_output = -self.min_forward_speed

            # network tables
            self.strafe_entry.set(pid_strafe_output)
            self.forward_entry.set(pid_forward_output)

        else:
            y_output = self.driver_controller.getY()
            x_output = self.driver_controller.getX()

        if self.vision_sub.front_v_entry == 1:
            self.drive_sub.drive(
                -wpimath.applyDeadband(
                    x_output, OIConstants.kDriveDeadband
                ),
                -wpimath.applyDeadband(
                    y_output, OIConstants.kDriveDeadband
                ),
                wpimath.applyDeadband(
                    self.driver_controller.getZ(), OIConstants.kDriveDeadband
                ),
                True,
                False,
            )
        else:
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
                self.driver_controller.getZ(), OIConstants.kDriveDeadband
            ),
            True,
            False,

        )
    def isFinished(self) -> bool:
        return False