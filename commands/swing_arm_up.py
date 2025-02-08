import commands2

from subsystems.swing_arm_subsystem import SwingArmSubsystem


class SwingArmUp(commands2.Command):

    def __init__(self, swing_arm_sub: SwingArmSubsystem) -> None:
        super().__init__()

        self.swing_arm_sub = swing_arm_sub
        self.addRequirements(self.swing_arm_sub)

    def execute(self) -> None:
        self.swing_arm_sub.set_swing_arm_speed(0.05)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.swing_arm_sub.set_swing_arm_speed(0)