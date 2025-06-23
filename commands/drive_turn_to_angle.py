import commands2
import wpilib
import wpimath.controller

from subsystems.drive_subsystem import DriveSubsystem


class DriveTurnToAngle(commands2.PIDCommand):
    max_speed = 0.5

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
    def initialize(self):
        self.getController().setTolerance(2)

    def isFinished(self) -> bool:
        return False