import cv2
import signal
import sys
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QApplication, QDialog

from autoloader_data import *
from autoloader_interfaces.msg import (
    Landmarks,
    MachineStatus,
    StartStopControl,
)
from autoloader_interfaces.srv import (
    ActuatorControl,
    MotionControl,
    RoutineControl,
    UpdateCoords,
)

from .src.gui import Interface


class InterfaceNode(Node):
    def __init__(self, interface: Interface) -> None:
        super().__init__("interface_node")

        self.landmarks = Landmarks()
        self.landmarks_updated = False

        self.run_status = RunState.STOP
        self.run_type = RunID.LOADING
        self.device_type = DeviceID.SMD

        self.head_curr = HeadID.LEFT
        self.position_curr_absolute = HeadPosition()
        self.position_curr_relative = HeadPosition()

        self.position_command = PositionCommand.MOVE
        self.smd_container = LandmarkID.SMD_WAFFLE
        self.to_container = LandmarkID.TO_WAFFLE
        self.jog_increment = 1.0

        self.interface = interface
        self.image_bridge = CvBridge()

        # Publishers
        self.start_stop_control_pub = self.create_publisher(
            msg_type=StartStopControl, topic="start_stop_control", qos_profile=10
        )

        # Subscribers
        self.coords_sub = self.create_subscription(
            msg_type=Landmarks,
            topic="coords",
            callback=self.coords_callback,
            qos_profile=10,
        )

        self.top_cam_sub = self.create_subscription(
            msg_type=Image,
            topic="top_camera",
            callback=lambda msg: self.camera_callback(msg, CameraID.TOP_CAMERA),
            qos_profile=10,
        )

        self.bot_cam_sub = self.create_subscription(
            msg_type=Image,
            topic="bot_camera",
            callback=lambda msg: self.camera_callback(msg, CameraID.BOT_CAMERA),
            qos_profile=10,
        )

        self.run_status_sub = self.create_subscription(
            msg_type=UInt32,
            topic="run_status",
            callback=self.run_status_callback,
            qos_profile=10,
        )

        self.machine_status_sub = self.create_subscription(
            msg_type=MachineStatus,
            topic="machine_status",
            callback=self.machine_status_callback,
            qos_profile=10,
        )

        # Clients
        self.actuator_cli = self.create_client(
            srv_type=ActuatorControl, srv_name="actuator_control"
        )

        self.motion_cli = self.create_client(
            srv_type=MotionControl, srv_name="motion_control"
        )

        self.routine_cli = self.create_client(
            srv_type=RoutineControl, srv_name="routine_control"
        )

        self.update_coords_cli = self.create_client(
            srv_type=UpdateCoords, srv_name="update_coords"
        )

        # Services
        self.homing_check_srv = self.create_service(
            srv_type=Trigger,
            srv_name="homing_check",
            callback=self.homing_check_callback,
        )

        self.starting_pos_srv = self.create_service(
            srv_type=Trigger,
            srv_name="starting_pos",
            callback=self.starting_pos_callback,
        )

    """
    Subscriber Callbacks
    """

    def coords_callback(self, msg: Landmarks) -> None:
        self.landmarks = msg
        self.landmarks_updated = True

    def camera_callback(self, image: Image, camera: CameraID) -> None:
        frame = self.image_bridge.imgmsg_to_cv2(image)
        _ = cv2.circle(
            frame,
            (int(frame.shape[1] / 2), int(frame.shape[0] / 2)),
            2,
            (255, 0, 0),
            -1,
        )
        _ = cv2.line(
            frame,
            (0, frame.shape[0] // 2),
            (frame.shape[1], frame.shape[0] // 2),
            (255, 0, 0),
            1,
        )
        _ = cv2.line(
            frame,
            (frame.shape[1] // 2, 0),
            (frame.shape[1] // 2, frame.shape[0]),
            (255, 0, 0),
            1,
        )

        frame_img = QImage(
            frame.data,
            frame.shape[1],
            frame.shape[0],
            frame.strides[0],
            QImage.Format.Format_RGB888,
        )

        match camera:
            case CameraID.TOP_CAMERA:
                self.interface.top_cam_view.setPixmap(
                    QPixmap.fromImage(frame_img.copy(self.interface.cropping_rect))
                )
            case CameraID.BOT_CAMERA:
                self.interface.bot_cam_view.setPixmap(
                    QPixmap.fromImage(frame_img.copy(self.interface.cropping_rect))
                )

    def run_status_callback(self, status) -> None:
        self.run_status = RunState(status.data)
        match self.run_status:
            case RunState.START:
                self.interface.status_label.setText("RUNNING")
                self.interface.status_label.setStyleSheet(
                    "background-color: rgb(46, 194, 126)"
                )
            case RunState.PAUSE:
                self.interface.status_label.setText("PAUSED")
                self.interface.status_label.setStyleSheet(
                    "background-color: rgb(246, 211, 45)"
                )
            case RunState.STOP:
                self.interface.status_label.setText("IDLE")
                self.interface.status_label.setStyleSheet(
                    "background-color: rgb(230, 25, 75)"
                )

    def machine_status_callback(self, msg: MachineStatus) -> None:
        self.position_curr_absolute.x = msg.position_absolute[0]
        self.position_curr_absolute.y = msg.position_absolute[1]
        self.position_curr_absolute.z = msg.position_absolute[2]

        self.position_curr_relative.x = msg.position_relative[0]
        self.position_curr_relative.y = msg.position_relative[1]
        self.position_curr_relative.z = msg.position_relative[2]

        self.interface.current_position_label.setText(
            f"X: {self.position_curr_absolute.x:.2f}, Y: {self.position_curr_absolute.y:.2f}, Z: {self.position_curr_absolute.z:.2f}"
        )

    def homing_check_callback(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:

        self.interface.homing_check_dialog.exec_()

        if self.interface.homing_check_dialog.result() == QDialog.DialogCode.Accepted:
            response.success = True
        else:
            response.success = False

        self.interface.homing_check_dialog.close()

        return response

    def starting_pos_callback(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:

        self.interface.starting_pos_dialog.exec_()

        if self.interface.starting_pos_dialog.result() == QDialog.DialogCode.Accepted:
            response.success = True
        else:
            response.success = False

        self.interface.starting_pos_dialog.close()

        self.update_coords_cli.call_async(
            UpdateCoords.Request(
                landmark_id=self.interface.starting_pos_ui.input_grid.carrier.id,
                run_slots=self.interface.starting_pos_ui.input_grid.carrier.run_slots,
            )
        )
        self.update_coords_cli.call_async(
            UpdateCoords.Request(
                landmark_id=self.interface.starting_pos_ui.output_grid.carrier.id,
                run_slots=self.interface.starting_pos_ui.output_grid.carrier.run_slots,
            )
        )

        self.update_coords_cli.call_async(
            UpdateCoords.Request(
                landmark_id=self.interface.starting_pos_ui.input_grid.carrier.id,
                failed_slots=self.interface.starting_pos_ui.input_grid.carrier.failed_slots,
            )
        )
        self.update_coords_cli.call_async(
            UpdateCoords.Request(
                landmark_id=self.interface.starting_pos_ui.output_grid.carrier.id,
                failed_slots=self.interface.starting_pos_ui.output_grid.carrier.failed_slots,
            )
        )

        return response

    """
    Publishers
    """

    def run_control(self, command: RunState) -> None:
        msg = StartStopControl()
        msg.command = command

        match command:
            case RunState.START:
                msg.run_params = [self.run_type, self.device_type]
                self.interface.status_label.setText("RUNNING")
                self.interface.status_label.setStyleSheet(
                    "background-color: rgb(46, 194, 126)"
                )
            case RunState.PAUSE:
                self.interface.status_label.setText("PAUSED")
                self.interface.status_label.setStyleSheet(
                    "background-color: rgb(246, 211, 45)"
                )
            case RunState.STOP:
                self.interface.status_label.setText("IDLE")
                self.interface.status_label.setStyleSheet(
                    "background-color: rgb(230, 25, 75)"
                )

        self.start_stop_control_pub.publish(msg)
        self.run_status = command

    """
    Service Calls
    """

    def jog_move(self, direction: JogDirection) -> None:
        if self.run_status == RunState.START:
            return

        request = MotionControl.Request(move_type=MoveID.JOG, head_curr=self.head_curr)

        match direction:
            case JogDirection.FORWARD:
                request.y = self.jog_increment * -1
            case JogDirection.BACK:
                request.y = self.jog_increment
            case JogDirection.LEFT:
                request.x = self.jog_increment * -1
            case JogDirection.RIGHT:
                request.x = self.jog_increment
            case JogDirection.UP:
                request.z = self.jog_increment * -1
            case JogDirection.DOWN:
                request.z = self.jog_increment
            case JogDirection.CLOCKWISE:
                request.r = self.jog_increment * -1
            case JogDirection.COUNTER_CLOCKWISE:
                request.r = self.jog_increment

        self.motion_cli.call_async(request)

    def routine_move(self, routine: RoutineID) -> None:
        if self.run_status == RunState.START:
            return

        # TODO: Add support for Manual Control of Other Routines
        if (
            routine == RoutineID.IDLE
            or routine == RoutineID.PARK_Z
            or routine == RoutineID.ALIGN_MACHINE
            or routine == RoutineID.HOME
        ):
            request = RoutineControl.Request(routine=routine)
            self.routine_cli.call_async(request)

    def switch_head_move(self, head_selected: HeadID) -> None:
        if self.run_status == RunState.START:
            return

        # Ensure No Head is at risk of Collision
        self.motion_cli.call_async(
            MotionControl.Request(
                move_type=MoveID.ABSOLUTE,
                head_curr=head_selected,
                z=0.0,
            )
        )

        # Move Camera to Selected Head Position
        if head_selected == HeadID.LEFT or head_selected == HeadID.RIGHT:
            self.motion_cli.call_async(
                MotionControl.Request(
                    move_type=MoveID.ABSOLUTE,
                    head_curr=head_selected,
                    x=self.position_curr_absolute.x,
                    y=self.position_curr_absolute.y,
                )
            )
        # Move Selected Head to Camera Position
        elif head_selected == HeadID.CAMERA:
            self.motion_cli.call_async(
                MotionControl.Request(
                    move_type=MoveID.ABSOLUTE,
                    head_curr=head_selected,
                    x=self.position_curr_relative.x,
                    y=self.position_curr_relative.y,
                )
            )

        self.motion_cli.call_async(
            MotionControl.Request(
                move_type=MoveID.ABSOLUTE,
                head_curr=head_selected,
                z=25.0,  # Safe Probe Height
            )
        )

    def landmark_move_capture(self, landmark_id: LandmarkID) -> None:
        if self.run_status == RunState.START:
            return

        landmark = getattr(self.landmarks, landmark_id.lower())

        match self.position_command:
            case PositionCommand.MOVE:
                self.motion_cli.call_async(
                    MotionControl.Request(
                        move_type=MoveID.ABSOLUTE,
                        head_curr=self.head_curr,
                        z=0.0,
                    )
                )
                self.motion_cli.call_async(
                    MotionControl.Request(
                        move_type=MoveID.ABSOLUTE,
                        head_curr=self.head_curr,
                        x=landmark.x,
                        y=landmark.y,
                    )
                )
                self.motion_cli.call_async(
                    MotionControl.Request(
                        move_type=MoveID.ABSOLUTE,
                        head_curr=self.head_curr,
                        z=landmark.z,
                    )
                )
            case PositionCommand.CAPTURE:
                self.update_coords_cli.call_async(
                    UpdateCoords.Request(
                        landmark_id=landmark_id,
                        x=self.position_curr_relative.x,
                        y=self.position_curr_relative.y,
                        z=self.position_curr_relative.z,
                    )
                )

    def marker_move_capture(self, landmark_id: LandmarkID) -> None:
        if self.run_status == RunState.START:
            return

        landmark = getattr(self.landmarks, landmark_id.lower())
        markers = reshape(landmark.markers, 2)

        match self.position_command:
            case PositionCommand.MOVE:
                self.motion_cli.call_async(
                    MotionControl.Request(
                        move_type=MoveID.ABSOLUTE,
                        head_curr=self.head_curr,
                        z=0.0,
                    )
                )
                self.motion_cli.call_async(
                    MotionControl.Request(
                        move_type=MoveID.ABSOLUTE,
                        head_curr=self.head_curr,
                        x=markers[0][0],
                        y=markers[0][1],
                    )
                )
                self.motion_cli.call_async(
                    MotionControl.Request(
                        move_type=MoveID.ABSOLUTE,
                        head_curr=self.head_curr,
                        z=landmark.z,
                    )
                )
            case PositionCommand.CAPTURE:
                self.get_logger().warning(
                    "Unable to Capture Marker Position via GUI yet!"
                )

    def actuator_toggle(self, actuator: ActuatorID, active: bool) -> None:
        if self.run_status == RunState.START:
            return

        request = ActuatorControl.Request(actuator=actuator, active=active)
        self.actuator_cli.call_async(request)

    """
    Miscellaneous
    """

    def destroy_node(self) -> None:
        self.interface.close()
        self.interface.destroy()
        self.interface.homing_check_dialog.destroy()
        super().destroy_node()

    def signal_handler(self, *_) -> None:
        self.destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    gui = Interface()
    gui_node = InterfaceNode(gui)

    executor = MultiThreadedExecutor()
    executor.add_node(gui_node)

    thread = Thread(target=executor.spin)
    thread.start()
    signal.signal(signal.SIGINT, gui_node.signal_handler)

    try:
        # Interface Setup must be AFTER GUI Node has started spinning
        # Otherwise, Interface will never receive valid Landmarks data from Coords Node
        gui.setup(gui_node)
        gui.show()
        sys.exit(app.exec_())
    except:
        pass
    finally:
        executor.shutdown()
        thread.join()
        gui_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
