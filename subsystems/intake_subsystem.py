import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class IntakeSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.intake_motor = rev.SparkFlex(5, rev.SparkFlex.MotorType.kBrushless)

        self.intake_prox_sensor = wpilib.DigitalInput(0)

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        lift_table = nt_instance.getTable("intake_table")

        self.intake_prox_state = lift_table.getDoubleTopic("intake_prox_state").publish()

    def periodic(self):
        self.intake_prox_state.set(self.intake_prox_sensor.get())

    def set_intake_speed(self, speed):
        self.intake_motor.set(speed)
