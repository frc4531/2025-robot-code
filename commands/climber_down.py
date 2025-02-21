import commands2

from subsystems.climber_subsystem import ClimberSubsystem


class ClimberDown(commands2.Command):

    def __init__(self, climber_sub: ClimberSubsystem) -> None:
        super().__init__()

        self.climber_sub = climber_sub
        self.addRequirements(self.climber_sub)

    def execute(self) -> None:
        self.climber_sub.set_climber_speed(-0.2)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.climber_sub.set_climber_speed(0)