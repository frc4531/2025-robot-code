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

        self.front_x_sub = front_table.getDoubleTopic("tx").subscribe(0.0)
        self.front_y_sub = front_table.getDoubleTopic("ty").subscribe(0.0)
        self.front_a_sub = front_table.getDoubleTopic("ta").subscribe(0.0)
        self.front_v_sub = front_table.getDoubleTopic("tv").subscribe(0.0)
        self.front_id_sub = front_table.getDoubleTopic("tid").subscribe(0)

        self.back_x_sub = back_table.getDoubleTopic("tx").subscribe(0.0)
        self.back_y_sub = back_table.getDoubleTopic("ty").subscribe(0.0)
        self.back_a_sub = back_table.getDoubleTopic("ta").subscribe(0.0)
        self.back_v_sub = back_table.getDoubleTopic("tv").subscribe(0.0)
        self.back_id_sub = back_table.getDoubleTopic("tid").subscribe(0)

        self.front_x_entry = 0
        self.front_y_entry = 0
        self.front_a_entry = 0
        self.front_v_entry = 0
        self.front_id_entry = 0

        self.back_x_entry = 0
        self.back_y_entry = 0
        self.back_a_entry = 0
        self.back_v_entry = 0
        self.back_id_entry = 0

    def periodic(self):
        self.front_x_entry = self.front_x_sub.get()
        self.front_y_entry = self.front_y_sub.get()
        self.front_a_entry = self.front_a_sub.get()
        self.front_v_entry = self.front_v_sub.get()
        self.front_id_entry = self.front_id_sub.get()

        self.back_x_entry = self.back_x_sub.get()
        self.back_y_entry = self.back_y_sub.get()
        self.back_a_entry = self.back_a_sub.get()
        self.back_v_entry = self.back_v_sub.get()
        self.back_id_entry = self.back_id_sub.get()
