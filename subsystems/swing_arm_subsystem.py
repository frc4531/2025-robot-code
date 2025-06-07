import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class SwingArmSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.swing_arm_motor = rev.SparkFlex(3, rev.SparkFlex.MotorType.kBrushless)

        self.swing_arm_abs_encoder = self.swing_arm_motor.getAbsoluteEncoder()

        # define the wrist subsystem's network table
        nt_instance = ntcore.NetworkTableInstance.getDefault()
        swing_arm_table = nt_instance.getTable("swing_arm_table")

        self.swing_arm_position_entry = swing_arm_table.getDoubleTopic("swing_arm_position").publish()
        self.swing_arm_pid_output_entry = swing_arm_table.getDoubleTopic("swing_arm_pid_output").publish()
        self.swing_arm_current_output_entry = swing_arm_table.getDoubleTopic("swing_arm_current_output").publish()

    def periodic(self) -> None:
        self.swing_arm_position_entry.set(self.get_swing_arm_position())
        self.swing_arm_current_output_entry.set(self.get_swing_arm_current())

    def set_swing_arm_speed(self, speed):
        if speed > 0.8:
            speed = 0.8
        elif speed < -0.8:
            speed = -0.8
        else:
            speed = speed
        self.swing_arm_motor.set(speed)

    def get_swing_arm_position(self):
        return self.swing_arm_abs_encoder.getPosition()

    def get_swing_arm_current(self):
        return self.swing_arm_motor.getOutputCurrent()
