import commands2
import wpilib
import wpimath.controller

from subsystems.drive_subsystem import DriveSubsystem


class DriveTurnAround(commands2.PIDCommand):
    max_speed = 0.25

    def __init__(self, drive_sub: DriveSubsystem, target_angle: float) -> None:
        super().__init__(
            wpimath.controller.PIDController(0.05, 0, 0),
            # Close loop on absolute encoder
            lambda: drive_sub.get_heading(),
            # Set reference to target
            target_angle,
            # Pipe output to turn arm
            lambda output: drive_sub.drive(0, 0,
                                           min(self.max_speed, max(-self.max_speed, -output)),
                                           False, False),
            # Require the arm
            drive_sub

        )
        self.drive_sub = drive_sub
        self.target_angle = target_angle

    def initialize(self):
        self.getController().setTolerance(2)

    def isFinished(self) -> bool:
        if self.target_angle > 0:
            return self.drive_sub.get_heading() > 170 or self.drive_sub.get_heading() < -100
        elif self.target_angle < 0:
            return self.drive_sub.get_heading() < -170 or self.drive_sub.get_heading() > 100
        else:
            return False