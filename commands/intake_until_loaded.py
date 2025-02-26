import commands2

from subsystems.intake_subsystem import IntakeSubsystem


class IntakeUntilLoaded(commands2.Command):

    def __init__(self, intake_sub: IntakeSubsystem) -> None:
        super().__init__()

        self.intake_sub = intake_sub
        self.addRequirements(self.intake_sub)

        self.prox_has_been_false = False

    def execute(self) -> None:
        self.intake_sub.set_intake_speed(-0.2)

        if self.intake_sub.intake_prox_sensor.get():
            self.prox_has_been_false = True

    def isFinished(self) -> bool:
        return not self.intake_sub.intake_prox_sensor.get() and self.prox_has_been_false

    def end(self, interrupted: bool) -> None:
        self.intake_sub.set_intake_speed(0)