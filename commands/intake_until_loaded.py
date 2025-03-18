import ntcore
import commands2

from subsystems.intake_subsystem import IntakeSubsystem


class IntakeUntilLoaded(commands2.Command):

    def __init__(self, intake_sub: IntakeSubsystem) -> None:
        super().__init__()

        self.intake_sub = intake_sub
        self.addRequirements(self.intake_sub)

        self.prox_has_been_false = False

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        intake_table = nt_instance.getTable("intake_table")

        self.currently_grabbing = intake_table.getBooleanTopic("currently_grabbing").publish()
        self.new_game_piece = intake_table.getBooleanTopic("new_game_piece").publish()

    def initialize(self):
        self.new_game_piece.set(False)
        self.currently_grabbing.set(True)

    def execute(self) -> None:
        self.intake_sub.set_intake_speed(-0.95)

        if self.intake_sub.intake_prox_sensor.get():
            self.prox_has_been_false = True

    def isFinished(self) -> bool:
        return not self.intake_sub.intake_prox_sensor.get() and self.prox_has_been_false

    def end(self, interrupted: bool) -> None:
        self.intake_sub.set_intake_speed(0)
        self.new_game_piece.set(True)
        self.currently_grabbing.set(False)