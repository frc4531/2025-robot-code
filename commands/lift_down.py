import commands2

from subsystems.lift_subsystem import LiftSubsystem


class LiftDown(commands2.Command):

    def __init__(self, lift_sub: LiftSubsystem) -> None:
        super().__init__()

        self.lift_sub = lift_sub
        self.addRequirements(self.lift_sub)

    def execute(self) -> None:
        self.lift_sub.set_lift_speed(-0.4)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.lift_sub.set_lift_speed(0)