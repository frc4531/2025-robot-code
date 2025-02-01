import rev
import wpilib
from commands2 import SubsystemBase


class SwingArmSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.swing_arm_motor = rev.SparkFlex(3, rev.SparkFlex.MotorType.kBrushless)

        self.swing_arm_abs_encoder = self.swing_arm_motor.getExternalEncoder()

    def set_swing_arm_speed(self, speed):
        self.swing_arm_motor.set(speed)