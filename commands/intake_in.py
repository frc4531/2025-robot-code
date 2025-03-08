import commands2

from subsystems.intake_subsystem import IntakeSubsystem


class IntakeIn(commands2.Command):

    def __init__(self, intake_sub: IntakeSubsystem) -> None:
        super().__init__()

        self.intake_sub = intake_sub
        self.addRequirements(self.intake_sub)

    def execute(self) -> None:
        self.intake_sub.set_intake_speed(-0.5)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.intake_sub.set_intake_speed(0)