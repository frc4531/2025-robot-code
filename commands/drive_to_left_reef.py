import math

import commands2
import ntcore
import wpilib
import wpimath.controller

from constants import field_pos_constants
from constants.swerve_constants import OIConstants
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.vision_subsystem import VisionSubsystem


class DriveToLeftReef(commands2.Command):

    def __init__(self, drive_sub: DriveSubsystem, vision_sub: VisionSubsystem, stick: wpilib.Joystick) -> None:
        super().__init__()

        self.drive_sub = drive_sub
        self.vision_sub = vision_sub
        self.addRequirements(self.drive_sub, self.vision_sub)

        self.driver_controller = stick

        # Constants
        self.max_strafe_speed = field_pos_constants.FieldPIDControl.kMaxStrafeSpeed

        self.max_forward_speed = field_pos_constants.FieldPIDControl.kMaxForwardSpeed

        self.max_rotate_speed = 0.5

        self.strafe_set_point = 0
        self.forward_set_point = 0

        self.strafe_target_threshold = 0.1
        self.forward_target_threshold = 0.1

        # X Speed Controller
        self.strafe_controller = wpimath.controller.PIDController(field_pos_constants.FieldPIDControl.kPStrafe, 0.1, 0) # 0.004, 0.03
        # self.strafe_controller.setSetpoint(-18.6)
        # Y Speed Controller
        self.forward_controller = wpimath.controller.PIDController(field_pos_constants.FieldPIDControl.kPForward, 0.1, 0) # 0.015
        # self.forward_controller.setSetpoint(14)
        # Z Speed Controller
        self.rotate_controller = wpimath.controller.PIDController(0.03, 0, 0)

        # network tables
        # nt_instance = ntcore.NetworkTableInstance.getDefault()
        # reef_table = nt_instance.getTable("reef_table")
        #
        # self.strafe_entry = reef_table.getDoubleTopic("strafe_entry").publish()
        # self.forward_entry = reef_table.getDoubleTopic("forward_entry").publish()
        # self.is_april_tag_tracking = reef_table.getBooleanTopic("is_april_tag_tracking").publish()
        # self.april_tag_tracked = reef_table.getBooleanTopic("april_tag_tracked").publish()

    # def initialize(self):
        # self.is_april_tag_tracking.set(True)

    def execute(self) -> None:
        if self.vision_sub.avg_v_entry == 1:
            pid_strafe_output = self.strafe_controller.calculate(self.vision_sub.avg_x_cord, self.strafe_set_point)
            pid_forward_output = (self.forward_controller.calculate(self.vision_sub.avg_y_cord, self.forward_set_point))

            x_output = max(min(pid_strafe_output, self.max_strafe_speed), -self.max_strafe_speed)
            y_output = max(min(pid_forward_output, self.max_forward_speed), -self.max_forward_speed)

            # if 0 < y_output < self.min_strafe_speed:
            #     y_output = self.min_strafe_speed
            # elif -self.min_strafe_speed > y_output > 0:
            #     y_output = -self.min_strafe_speed
            #
            # if 0 < x_output < self.min_forward_speed:
            #     x_output = self.min_forward_speed
            # elif -self.min_forward_speed > x_output > 0:
            #     x_output = -self.min_forward_speed

            # network tables
            # self.strafe_entry.set(pid_strafe_output)
            # self.forward_entry.set(pid_forward_output)

            # START ROTATE BLOCK
            match self.vision_sub.avg_id_entry:
                case 6:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID6XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID6YLeft
                    target_angle = 60
                case 7:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID7XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID7YLeft
                    target_angle = 0
                case 8:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID8XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID8YLeft
                    target_angle = -60
                case 9:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID9XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID9YLeft
                    target_angle = -120
                case 10:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID10XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID10YLeft
                    target_angle = self.drive_sub.get_heading()
                case 11:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID11XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID11YLeft
                    target_angle = 120
                # Blue Reef
                case 17:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID17XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID17YLeft
                    target_angle = -60
                case 18:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID18XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID18YLeft
                    target_angle = 0
                case 19:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID19XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID19YLeft
                    target_angle = 60 #60
                case 20:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID20XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID20YLeft
                    target_angle = 120
                case 21:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID21XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID21YLeft
                    target_angle = self.drive_sub.get_heading()
                case 22:
                    self.strafe_set_point = field_pos_constants.FieldConstants.kID22XLeft
                    self.forward_set_point = field_pos_constants.FieldConstants.kID22YLeft
                    target_angle = -120
                case _:
                    target_angle = self.drive_sub.get_heading()

            # self.rotate_controller.setSetpoint(target_angle)
            pid_rotate_output = self.rotate_controller.calculate(self.drive_sub.get_heading(), target_angle)

            z_output = max(min(-pid_rotate_output, self.max_rotate_speed), -self.max_rotate_speed)

        else:
            y_output = self.driver_controller.getY()
            x_output = self.driver_controller.getX()
            z_output = self.driver_controller.getZ()

        if self.vision_sub.avg_v_entry == 1:
            self.drive_sub.drive(
                wpimath.applyDeadband(
                    x_output, OIConstants.kDriveDeadband
                ),
                wpimath.applyDeadband(
                    y_output, OIConstants.kDriveDeadband
                ),
                wpimath.applyDeadband(
                    z_output, OIConstants.kDriveDeadband
                ),
                False,
                False,
            )
        else:
            self.drive_sub.drive(
                wpimath.applyDeadband(
                    (-self.driver_controller.getY() * math.cos(self.drive_sub.get_heading() * (math.pi / 180))) +
                    (self.driver_controller.getX() * math.sin(self.drive_sub.get_heading() * (math.pi / 180))),
                    OIConstants.kDriveDeadband
                ),
                -wpimath.applyDeadband(
                    (self.driver_controller.getY() * math.sin(self.drive_sub.get_heading() * (math.pi / 180))) +
                    (self.driver_controller.getX() * math.cos(self.drive_sub.get_heading() * (math.pi / 180))),
                    OIConstants.kDriveDeadband
                ),
                -wpimath.applyDeadband(
                    self.driver_controller.getZ(), OIConstants.kDriveDeadband
                ),
                False,
                False,
            )

        # LED Boolean Logic
        # if (self.strafe_target_threshold + self.strafe_set_point) > self.vision_sub.front_x_sub > (self.strafe_set_point - self.strafe_target_threshold):
        #     self.april_tag_tracked.set(True)
        # else:
        #     self.april_tag_tracked.set(False)

    # def end(self):
            # self.is_april_tag_tracking.set(False)

    def isFinished(self) -> bool:
        return False