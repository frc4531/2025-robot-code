import wpilib.sysid
import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class ClimberSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

    def set_climber_speed(self, speed):
        self.climber_motor.set(speed)

