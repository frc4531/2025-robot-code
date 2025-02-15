import ntcore
import rev
import wpilib
from commands2 import SubsystemBase
import ntcore


class WristSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.wrist_motor = rev.SparkFlex(4, rev.SparkFlex.MotorType.kBrushless)

        self.wrist_abs_encoder = self.wrist_motor.getAbsoluteEncoder()

        # define the wrist subsystem's network table
        nt_instance = ntcore.NetworkTableInstance.getDefault()
        wrist_table = nt_instance.getTable("wrist_table")

        self.wrist_position_entry = wrist_table.getDoubleTopic("wrist_position").publish()
        self.wrist_pid_output_entry = wrist_table.getDoubleTopic("wrist_pid_output").publish()


    def periodic(self) -> None:
        self.wrist_position_entry.set(self.get_wrist_position())

    def set_wrist_speed(self, speed):
        self.wrist_motor.set(speed)

    def get_wrist_position(self):
        return self.wrist_abs_encoder.getPosition()
