import commands2
import wpilib
import wpimath.controller

from subsystems.lift_subsystem import LiftSubsystem


class SwingArmToPosition(commands2.PIDCommand):

    def __init__(self, lift_sub: LiftSubsystem, target_position) -> None:
        super().__init__(
            wpimath.controller.PIDController(7.5, 0, 0),
            # Close loop on absolute encoder
            lambda: lift_sub.get_distance(),
            # Set reference to target
            target_position,
            # Pipe output to turn arm
            lambda output: lift_sub.set_lift_speed(output),
            # Require the arm
            lift_sub
        )

    def isFinished(self) -> bool:
        return False