import commands2
import rev
from rev import SparkFlexConfig, SparkBase

from subsystems.lift_subsystem import LiftSubsystem


class LiftToPos2(commands2.Command):

    def __init__(self, lift_sub: LiftSubsystem, target_position) -> None:
        super().__init__()

        self.target_position = target_position

        self.lift_sub = lift_sub
        self.addRequirements(self.lift_sub)

    def execute(self) -> None:
        self.lift_sub.left_pid_controller.setReference(self.target_position, rev.SparkBase.ControlType.kPosition)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.lift_sub.set_lift_speed(0)
