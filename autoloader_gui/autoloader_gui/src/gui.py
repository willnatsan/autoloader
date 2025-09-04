from PySide6.QtWidgets import QMainWindow, QDialog, QMessageBox
from PySide6.QtCore import QRect, QPoint

from rclpy.node import Node

from autoloader_data import *

from .runner_ui import Ui_MVP
from .homing_check_ui import Ui_Homing
from .starting_positions_ui import Ui_Starting_Pos


class Interface(QMainWindow, Ui_MVP):
    def __init__(
        self,
    ) -> None:
        super().__init__()
        self.setWindowTitle("AutoLoader GUI")
        self.setupUi(self)

        self.ros_node = None

        # QRect used for cropping camera images to desired size for GUI
        self.video_height = 550
        self.video_width = 700
        self.cropping_rect = QRect(
            QPoint(
                int(1280 / 2 - self.video_width / 2),
                int(720 / 2 - self.video_height / 2),
            ),
            QPoint(
                int(1280 / 2 + self.video_width / 2),
                int(720 / 2 + self.video_height / 2),
            ),
        )

        self.homing_check_dialog = QDialog(parent=self)
        self.homing_check_ui = Ui_Homing()
        self.homing_check_dialog.setWindowTitle("Homing Check")

        self.starting_pos_dialog = QDialog(parent=self)
        self.starting_pos_ui = Ui_Starting_Pos()
        self.starting_pos_dialog.setWindowTitle("Set Input / Output Positions")

    def setup(self, node: Node) -> None:
        self.ros_node = node

        # Waiting for valid Landmarks data from Coords Node before Setup
        while not self.ros_node.landmarks_updated:
            continue

        self.homing_check_ui.setupUi(self.homing_check_dialog)
        self.starting_pos_ui.setupUi(self.starting_pos_dialog, self.ros_node.landmarks)

        # Run Management Tab
        self.device_type_box.currentIndexChanged.connect(self.assign_device_type)
        self.run_type_box.currentIndexChanged.connect(self.assign_run_type)

        self.start_button.clicked.connect(
            lambda: self.ros_node.run_control(RunState.START)
        )
        self.pause_button.clicked.connect(
            lambda: self.ros_node.run_control(RunState.PAUSE)
        )
        self.stop_button.clicked.connect(
            lambda: self.ros_node.run_control(RunState.STOP)
        )

        # Manual Control Tab
        self.head_combo_box.currentIndexChanged.connect(self.assign_head)
        self.position_combo_box.currentIndexChanged.connect(
            self.assign_position_command
        )
        self.jog_increment_slider.valueChanged.connect(self.assign_jog_increment)

        self.back_button.clicked.connect(
            lambda: self.ros_node.jog_move(JogDirection.BACK)
        )
        self.forward_button.clicked.connect(
            lambda: self.ros_node.jog_move(JogDirection.FORWARD)
        )
        self.left_button.clicked.connect(
            lambda: self.ros_node.jog_move(JogDirection.LEFT)
        )
        self.right_button.clicked.connect(
            lambda: self.ros_node.jog_move(JogDirection.RIGHT)
        )
        self.up_button.clicked.connect(lambda: self.ros_node.jog_move(JogDirection.UP))
        self.down_button.clicked.connect(
            lambda: self.ros_node.jog_move(JogDirection.DOWN)
        )
        self.rotate_cw_button.clicked.connect(
            lambda: self.ros_node.jog_move(JogDirection.CLOCKWISE)
        )
        self.rotate_ccw_button.clicked.connect(
            lambda: self.ros_node.jog_move(JogDirection.COUNTER_CLOCKWISE)
        )

        self.idle_button.clicked.connect(
            lambda: self.ros_node.routine_move(RoutineID.IDLE)
        )
        self.park_z_button.clicked.connect(
            lambda: self.ros_node.routine_move(RoutineID.PARK_Z)
        )
        self.align_visual_button.clicked.connect(
            lambda: self.ros_node.routine_move(RoutineID.ALIGN_MACHINE)
        )
        self.home_button.clicked.connect(
            lambda: self.ros_node.routine_move(RoutineID.HOME)
        )

        self.homing_check_ui.dialog_button_box.accepted.connect(
            self.homing_check_dialog.accept
        )
        self.homing_check_ui.dialog_button_box.rejected.connect(
            self.homing_check_dialog.reject
        )

        self.camera_move_button.clicked.connect(
            lambda: self.ros_node.switch_head_move(HeadID.CAMERA)
        )
        self.head_left_move_button.clicked.connect(
            lambda: self.ros_node.switch_head_move(HeadID.LEFT)
        )
        self.head_right_move_button.clicked.connect(
            lambda: self.ros_node.switch_head_move(HeadID.RIGHT)
        )

        self.position_fiducial_button.clicked.connect(
            lambda: self.ros_node.landmark_move_capture(LandmarkID.HOMING_FIDUCIAL)
        )
        self.position_bottom_camera_button.clicked.connect(
            lambda: self.ros_node.landmark_move_capture(LandmarkID.BOTTOM_CAMERA)
        )

        self.smd_combo_box.currentIndexChanged.connect(self.assign_smd_container)
        self.to_combo_box.currentIndexChanged.connect(self.assign_to_container)

        self.position_smd_starting_button.clicked.connect(
            lambda: self.ros_node.landmark_move_capture(self.ros_node.smd_container)
        )
        self.position_smd_fiducial_button.clicked.connect(
            lambda: self.ros_node.marker_move_capture(self.ros_node.smd_container)
        )
        self.position_to_starting_button.clicked.connect(
            lambda: self.ros_node.landmark_move_capture(self.ros_node.to_container)
        )
        self.position_to_fiducial_button.clicked.connect(
            lambda: self.ros_node.marker_move_capture(self.ros_node.to_container)
        )

        self.top_light_on_button.clicked.connect(
            lambda: self.ros_node.actuator_toggle(ActuatorID.TOP_LIGHT, True)
        )
        self.top_light_off_button.clicked.connect(
            lambda: self.ros_node.actuator_toggle(ActuatorID.TOP_LIGHT, False)
        )
        self.bot_light_on_button.clicked.connect(
            lambda: self.ros_node.actuator_toggle(ActuatorID.BOT_LIGHT, True)
        )
        self.bot_light_off_button.clicked.connect(
            lambda: self.ros_node.actuator_toggle(ActuatorID.BOT_LIGHT, False)
        )
        self.left_actuator_on_button.clicked.connect(
            lambda: self.ros_node.actuator_toggle(ActuatorID.LEFT_ACTUATOR, True)
        )
        self.left_actuator_off_button.clicked.connect(
            lambda: self.ros_node.actuator_toggle(ActuatorID.LEFT_ACTUATOR, False)
        )
        self.right_actuator_on_button.clicked.connect(
            lambda: self.ros_node.actuator_toggle(ActuatorID.RIGHT_ACTUATOR, True)
        )
        self.right_actuator_off_button.clicked.connect(
            lambda: self.ros_node.actuator_toggle(ActuatorID.RIGHT_ACTUATOR, False)
        )

        self.starting_pos_ui.select_failures_checkbox.stateChanged.connect(
            self.toggle_starting_positions_selection
        )
        self.starting_pos_ui.reset_positions_button.clicked.connect(
            self.reset_positions
        )

    def update_position_grids(self) -> None:
        match self.ros_node.device_type:
            case DeviceID.SMD:
                input_carrier = self.ros_node.landmarks.smd_waffle
                output_carrier = self.ros_node.landmarks.smd_carrier
            case DeviceID.TO:
                input_carrier = self.ros_node.landmarks.to_waffle
                output_carrier = self.ros_node.landmarks.to_carrier
            case DeviceID.DIE:
                return

        if self.ros_node.run_type == RunID.UNLOADING:
            input_carrier, output_carrier = output_carrier, input_carrier

        self.starting_pos_ui.input_grid.switch_carrier(input_carrier, output_carrier)
        self.starting_pos_ui.output_grid.switch_carrier(output_carrier)

    def assign_device_type(self, device_type: int) -> None:
        self.ros_node.device_type = DeviceID(device_type)
        self.update_position_grids()

    def assign_run_type(self, run_type: int) -> None:
        self.ros_node.run_type = RunID(run_type)
        self.update_position_grids()

    def assign_head(self, head: int) -> None:
        self.ros_node.head_curr = HeadID(head)

    def assign_position_command(self, position_command: int) -> None:
        self.ros_node.position_command = PositionCommand(position_command)

    def assign_jog_increment(self, jog_increment: int) -> None:
        self.ros_node.jog_increment = 0.01 * 10.0**jog_increment

    def assign_smd_container(self, carrier: int) -> None:
        smd_container = (LandmarkID.SMD_WAFFLE, LandmarkID.SMD_CARRIER)[carrier]
        self.ros_node.smd_container = smd_container

    def assign_to_container(self, carrier: int) -> None:
        to_container = (LandmarkID.TO_WAFFLE, LandmarkID.TO_CARRIER)[carrier]
        self.ros_node.to_container = to_container

    def toggle_starting_positions_selection(self) -> None:
        if self.starting_pos_ui.select_failures_checkbox.isChecked():
            self.starting_pos_ui.input_grid.select_failures = True
            self.starting_pos_ui.output_grid.select_failures = True
        else:
            self.starting_pos_ui.input_grid.select_failures = False
            self.starting_pos_ui.output_grid.select_failures = False

    def reset_positions(self) -> None:
        self.starting_pos_ui.input_grid.reset()
        self.starting_pos_ui.output_grid.reset()

    def show_popup(self, popup_type: PopUpID, msg: str, info: str = "") -> None:
        dialog = QMessageBox()
        dialog.setWindowTitle(popup_type)
        dialog.setText(msg)
        dialog.setInformativeText(info)
        dialog.setStandardButtons(QMessageBox.StandardButton.Ok)

        match popup_type:
            case PopUpID.INFO:
                dialog.setIcon(QMessageBox.Icon.Information)
            case PopUpID.WARNING:
                dialog.setIcon(QMessageBox.Icon.Warning)
            case PopUpID.ERROR:
                dialog.setIcon(QMessageBox.Icon.Critical)

        dialog.exec_()
        dialog.close()
