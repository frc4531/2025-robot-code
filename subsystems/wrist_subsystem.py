import rev
import wpilib
from commands2 import SubsystemBase


class WristSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.wrist_motor = rev.SparkFlex(4, rev.SparkFlex.MotorType.kBrushless)

        self.wrist_abs_encoder = self.wrist_motor.getExternalEncoder()

    def set_wrist_speed(self, speed):
        self.wrist_motor.set(speed)