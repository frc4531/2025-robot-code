import ntcore
import wpilib

from networktables import NetworkTables
from commands2 import SubsystemBase


class VisionSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()
        self.inst = ntcore.NetworkTableInstance.getDefault()
        left_table = self.inst.getTable("limelight-left")
        right_table = self.inst.getTable("limelight-right")
        vision_table = self.inst.getTable("vision_table")

        self.left_x_sub = left_table.getDoubleTopic("tx").subscribe(0.0)
        self.left_y_sub = left_table.getDoubleTopic("ty").subscribe(0.0)
        self.left_a_sub = left_table.getDoubleTopic("ta").subscribe(0.0)
        self.left_v_sub = left_table.getDoubleTopic("tv").subscribe(0.0)
        self.left_id_sub = left_table.getDoubleTopic("tid").subscribe(0)
        self.left_blue_pos_sub = left_table.getFloatArrayTopic("botpose_orb_wpiblue").subscribe([0.0, 0.0])

        self.right_x_sub = right_table.getDoubleTopic("tx").subscribe(0.0)
        self.right_y_sub = right_table.getDoubleTopic("ty").subscribe(0.0)
        self.right_a_sub = right_table.getDoubleTopic("ta").subscribe(0.0)
        self.right_v_sub = right_table.getDoubleTopic("tv").subscribe(0.0)
        self.right_id_sub = right_table.getDoubleTopic("tid").subscribe(0)
        self.right_blue_pos_sub = right_table.getFloatArrayTopic("botpose_orb_wpiblue").subscribe([0.0, 0.0])

        self.left_x_entry = 0
        self.left_y_entry = 0
        self.left_a_entry = 0
        self.left_v_entry = 0
        self.left_id_entry = 0
        self.left_blue_pos = 0

        self.right_x_entry = 0
        self.right_y_entry = 0
        self.right_a_entry = 0
        self.right_v_entry = 0
        self.right_id_entry = 0
        self.right_blue_pos = 0

        self.avg_y_cord = 0
        self.avg_x_cord = 0

        self.avg_y_cord_entry = vision_table.getFloatTopic("avg_y_cord").publish()
        self.avg_x_cord_entry = vision_table.getFloatTopic("avg_x_cord").publish()

    def periodic(self):
        self.left_x_entry = self.left_x_sub.get()
        self.left_y_entry = self.left_y_sub.get()
        self.left_a_entry = self.left_a_sub.get()
        self.left_v_entry = self.left_v_sub.get()
        self.left_id_entry = self.left_id_sub.get()
        self.left_blue_pos = self.left_blue_pos_sub.get()

        self.right_x_entry = self.right_x_sub.get()
        self.right_y_entry = self.right_y_sub.get()
        self.right_a_entry = self.right_a_sub.get()
        self.right_v_entry = self.right_v_sub.get()
        self.right_id_entry = self.right_id_sub.get()
        self.right_blue_pos = self.right_blue_pos_sub.get()

        # Avg Pos Estimator
        if self.left_v_entry == 1 and self.right_v_entry == 1:
            self.avg_y_cord = (self.left_blue_pos[1] + self.right_blue_pos[1]) / 2
            self.avg_x_cord = (self.left_blue_pos[0] + self.right_blue_pos[0]) / 2
        elif self.left_v_entry == 1 and self.right_v_entry == 0:
            self.avg_y_cord = self.left_blue_pos[1]
            self.avg_x_cord = self.left_blue_pos[0]
        elif self.left_v_entry == 0 and self.right_v_entry == 1:
            self.avg_y_cord = self.right_blue_pos[1]
            self.avg_x_cord = self.right_blue_pos[0]
        else:
            self.avg_y_cord = 0
            self.avg_x_cord = 0

        self.avg_y_cord_entry.set(self.avg_y_cord)
        self.avg_x_cord_entry.set(self.avg_x_cord)
