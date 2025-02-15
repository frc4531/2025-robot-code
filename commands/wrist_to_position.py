import commands2
import wpilib
import wpimath.controller

from subsystems.wrist_subsystem import WristSubsystem


class WristToPosition(commands2.PIDCommand):

    def __init__(self, wrist_sub: WristSubsystem, target_position) -> None:
        super().__init__(
            wpimath.controller.PIDController(1.5, 0, 0),
            # Close loop on absolute encoder
            lambda: wrist_sub.get_wrist_position(),
            # Set reference to target
            target_position,
            # Pipe output to turn arm
            lambda output: wrist_sub.wrist_pid_output_entry.set(output),
            # Require the arm
            wrist_sub
        )

    def isFinished(self) -> bool:
        return False