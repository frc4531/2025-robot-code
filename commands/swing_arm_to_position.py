import commands2
import wpilib
import wpimath.controller

from subsystems.swing_arm_subsystem import SwingArmSubsystem


class SwingArmToPosition(commands2.PIDCommand):

    def __init__(self, swing_arm_sub: SwingArmSubsystem, target_position) -> None:
        super().__init__(
            wpimath.controller.PIDController(5, 0, 0),
            # Close loop on absolute encoder
            lambda: swing_arm_sub.get_swing_arm_position(),
            # Set reference to target
            target_position,
            # Pipe output to turn arm
            lambda output: swing_arm_sub.set_swing_arm_speed(-output),
            # Require the arm
            swing_arm_sub
        )

    def isFinished(self) -> bool:
        return False