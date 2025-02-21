import ntcore
import rev
import wpilib
from commands2 import SubsystemBase
import ntcore


class ClimberSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.climber_motor = rev.SparkFlex(6, rev.SparkFlex.MotorType.kBrushless)

    def set_climber_speed(self, speed):
        self.climber_motor.set(speed)
