import commands2

from subsystems.swing_arm_subsystem import SwingArmSubsystem


class LiftUp(commands2.Command):

    def __init__(self, swing_arm_sub: SwingArmSubsystem) -> None:
        super().__init__()

        self.swing_arm_sub = swing_arm_sub
        self.addRequirements(self.lift_sub)

    def execute(self) -> None:
        self.lift_sub.set_lift_speed(0.05)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.lift_sub.set_lift_speed(0)