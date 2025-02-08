import commands2

from subsystems.wrist_subsystem import WristSubsystem


class WristDown(commands2.Command):

    def __init__(self, wrist_sub: WristSubsystem) -> None:
        super().__init__()

        self.wrist_sub = wrist_sub
        self.addRequirements(self.wrist_sub)

    def execute(self) -> None:
        self.wrist_sub.set_wrist_speed(-0.05)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.wrist_sub.set_wrist_speed(0)