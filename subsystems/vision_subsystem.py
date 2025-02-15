import ntcore
import wpilib

from networktables import NetworkTables
from commands2 import SubsystemBase


class VisionSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()
        self.inst = ntcore.NetworkTableInstance.getDefault()
        front_table = self.inst.getTable("limelight-front")
        back_table = self.inst.getTable("limelight-back")

        self.front_x_entry = front_table.getDoubleTopic("tx").subscribe(0.0)
        self.front_y_entry = front_table.getDoubleTopic("ty").subscribe(0.0)
        self.front_a_entry = front_table.getDoubleTopic("ta").subscribe(0.0)
        self.front_v_entry = front_table.getDoubleTopic("tv").subscribe(0.0)
        self.front_id_entry = front_table.getDoubleTopic("tid").subscribe(0)

        self.back_x_entry = back_table.getDoubleTopic("tx").subscribe(0.0)
        self.back_y_entry = back_table.getDoubleTopic("ty").subscribe(0.0)
        self.back_a_entry = back_table.getDoubleTopic("ta").subscribe(0.0)
        self.back_v_entry = back_table.getDoubleTopic("tv").subscribe(0.0)
        self.back_id_entry = back_table.getDoubleTopic("tid").subscribe(0)
