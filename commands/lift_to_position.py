import commands2
import wpilib
import wpimath.controller

from subsystems.lift_subsystem import LiftSubsystem


class LiftToPosition(commands2.PIDCommand):

    def __init__(self, lift_sub: LiftSubsystem, target_position) -> None:
        super().__init__(
            wpimath.controller.PIDController(0.045, 0, 0),
            # Close loop on absolute encoder
            lambda: lift_sub.get_lift_position(),
            # Set reference to target
            target_position,
            # Pipe output to turn arm
            lambda output: lift_sub.set_lift_speed(output) and lift_sub.lift_pid_output_entry(output),
            # Require the arm
            lift_sub
        )

    def isFinished(self) -> bool:
        return False