import time

from rclpy.node import Node

from autoloader_data import *
from autoloader_interfaces.msg import Landmarks
from autoloader_interfaces.srv import RoutineControl, UpdateCoords

from .src._pnp import PnP
from .src.utils import find_skew, correct_skew, kasa_circle_estimation


class RoutineNode(Node):
    def __init__(self, pnp_machine: PnP) -> None:
        super().__init__("routine_node")

        self.pnp = pnp_machine
        self.landmarks = Landmarks()
        self.landmarks_updated = False

        # Subscribers
        self.coords_sub = self.create_subscription(
            msg_type=Landmarks,
            topic="coords",
            callback=self.coords_callback,
            qos_profile=10,
        )

        # Clients
        self.update_coords_cli = self.create_client(
            srv_type=UpdateCoords, srv_name="update_coords"
        )

        # Services
        self.routine_srv = self.create_service(
            srv_type=RoutineControl,
            srv_name="routine_control",
            callback=self.routine_callback,
        )

    def coords_callback(self, msg: Landmarks) -> None:
        self.landmarks = msg
        self.landmarks_updated = True

    def routine_callback(
        self, request: RoutineControl.Request, response: RoutineControl.Response
    ) -> RoutineControl.Response:

        routine = RoutineID(request.routine)

        if not self.landmarks_updated:
            self.get_logger().warning("Can't call ANY Routine w/ No Landmark Data")
            response.success = False
            return response

        if routine != RoutineID.HOME and not self.pnp.homed:
            self.get_logger().warning("Can't call THIS Routine when Machine not Homed")
            response.success = False
            return response

        match routine:
            case RoutineID.HOME:
                response.success = self.pnp.home()

            case RoutineID.PARK_Z:
                response.success = self.pnp.move(z=0.0)

            case RoutineID.IDLE:
                response.success = self.idle()

            case RoutineID.ALIGN_MACHINE:
                response.success = self.align_machine()

            case RoutineID.ALIGN_EE:
                head = HeadID(request.head_curr)
                response.success = self.align_ee(head)

            case RoutineID.ALIGN_CARRIER:
                carrier = LandmarkID(request.carrier)
                response.success = self.align_carrier(carrier)

            case RoutineID.ALIGN_DEVICE:
                head = HeadID(request.head_curr)
                device = DeviceID(request.device)
                response.success, response.offsets = self.align_device(head, device)

            case RoutineID.PICK:
                head = HeadID(request.head_curr)
                carrier = LandmarkID(request.carrier)
                response.success = self.pick(head, carrier, request.device_number)

            case RoutineID.PLACE:
                head = HeadID(request.head_curr)
                carrier = LandmarkID(request.carrier)
                response.success = self.place(
                    head, carrier, request.device_number, request.offsets
                )

        return response

    def idle(self) -> bool:
        for actuator in self.pnp.actuators.values():
            if not actuator.off():
                return False

        if not self.pnp.move(z=0.0, r=0.0):
            return False

        if not self.pnp.move(x=0.5, y=450):
            return False

        return True

    def align_machine(self) -> bool:
        self.pnp.move(
            x=self.landmarks.homing_fiducial.x, y=self.landmarks.homing_fiducial.y
        )
        self.pnp.light_top.on()

        fiducial_offset = Offset()
        successes = 0

        while successes < 5:
            self.pnp.move(x=fiducial_offset.x, y=fiducial_offset.y, jog=True)

            result, fiducial_offset = self.pnp.get_fiducial_offset()
            if not result:
                self.get_logger().error("Failed to get Fiducial Offset")
                return False

            if abs(fiducial_offset.x) < 0.016 and abs(fiducial_offset.y) < 0.016:
                successes += 1
            else:
                successes = 0

        self.pnp.light_top.off()
        self.pnp.reset_position(
            HeadPosition(
                x=self.landmarks.homing_fiducial.x - fiducial_offset.x,
                y=self.landmarks.homing_fiducial.y - fiducial_offset.y,
            )
        )

        return True

    def align_ee(self, head_id: HeadID) -> bool:
        self.pnp.move(
            x=self.landmarks.bottom_camera.x,
            y=self.landmarks.bottom_camera.y,
            r=0.0,
            head=head_id,
        )
        self.pnp.move(z=self.landmarks.bottom_camera.z, head=head_id)

        self.pnp.light_bot.on()

        while True:
            ret, offset = self.pnp.get_ee_offset(head_id)
            if ret:
                break

        self.pnp.head_offsets[head_id].x += offset.x
        self.pnp.head_offsets[head_id].y += offset.y

        self.pnp.light_bot.off()

        self.pnp.move(z=0.0, r=0.0, head=head_id)

        return True

    def align_carrier(self, carrier_id: LandmarkID) -> bool:
        if (
            carrier_id == LandmarkID.HOMING_FIDUCIAL
            or carrier_id == LandmarkID.BOTTOM_CAMERA
        ):
            self.get_logger().error("Invalid Carrier for Alignment!")
            return False

        carrier = getattr(self.landmarks, carrier_id.lower())
        if not carrier.markers:
            self.get_logger().error("Markers not set for this Carrier!")
            return False

        self.pnp.light_top.on()

        marker_points_expected = reshape(carrier.markers, 2)
        marker_points = []

        for marker_x, marker_y in marker_points_expected:
            self.pnp.move(x=marker_x, y=marker_y)
            time.sleep(1)

            result, marker_offset = self.pnp.get_fiducial_offset(carrier.marker_radius)
            if not result:
                self.get_logger().error("Failed to get Marker Offset")
                return False

            marker_points.append(
                (marker_x + marker_offset.x, marker_y + marker_offset.y)
            )

        self.pnp.light_top.off()

        carrier.skew = flatten(find_skew(marker_points_expected, marker_points))

        self.update_coords_cli.call_async(
            UpdateCoords.Request(landmark_id=carrier_id, skew=carrier.skew)
        )

        return True

    def align_device(
        self, head_id: HeadID, device_id: DeviceID
    ) -> tuple[bool, list[float, float]]:

        self.pnp.move(
            x=self.landmarks.bottom_camera.x,
            y=self.landmarks.bottom_camera.y,
            r=0.0,
            head=head_id,
        )
        self.pnp.move(
            z=self.landmarks.bottom_camera.z - DEVICE_HEIGHTS[device_id], head=head_id
        )

        self.pnp.light_bot.on(a=80)

        result, device_orientation = self.pnp.get_device_orientation(device_id)
        if not result:
            self.get_logger().error("Failed to get Device Orientation")
            return False

        self.pnp.move(r=device_orientation, jog=True, head=head_id)

        self.pnp.light_bot.on(a=255)

        result, device_offset = self.pnp.get_device_offset(device_id)
        if not result:
            self.get_logger().error(
                "Failed to get Device Offset -> For Rotation Correction"
            )
            return False, [device_offset.x, device_offset.y]

        self.pnp.move(r=device_offset.r, jog=True, head=head_id)
        time.sleep(0.1)

        result, device_offset = self.pnp.get_device_offset(device_id)
        if not result:
            self.get_logger().error(
                "Failed to get Device Offset -> For Translation Correction"
            )
            return False, [device_offset.x, device_offset.y]

        self.pnp.light_bot.off()

        self.pnp.move(z=0.0, head=head_id)

        return True, [device_offset.x, device_offset.y]

    def pick(self, head_id: HeadID, carrier_id: LandmarkID, device_number: int) -> bool:
        if (
            carrier_id == LandmarkID.HOMING_FIDUCIAL
            or carrier_id == LandmarkID.BOTTOM_CAMERA
        ):
            self.get_logger().error("Invalid Carrier for Pick!")
            return False

        carrier = getattr(self.landmarks, carrier_id.lower())
        run_slots = reshape(carrier.run_slots, 2)
        skew = reshape(carrier.skew, 3)

        input_col, input_row = run_slots[device_number]

        match head_id:
            case HeadID.LEFT:
                actuator = self.pnp.actuator_left
            case HeadID.RIGHT:
                actuator = self.pnp.actuator_right
            case HeadID.CAMERA:
                self.get_logger().error("Can't Pick with Camera!")
                return False

        input_x = carrier.x + carrier.x_increment * input_col
        input_y = carrier.y + carrier.y_increment * input_row
        input_r = 0.0

        if carrier.markers:
            input_x, input_y, input_r = correct_skew(input_x, input_y, skew)

        self.pnp.move(x=input_x, y=input_y, r=0.0, head=head_id)
        self.pnp.move(r=input_r, jog=True, head=head_id)
        self.pnp.move(z=carrier.z, head=head_id)

        actuator.on()

        self.pnp.move(z=0.0, head=head_id)

        return actuator.part_detected()

    # TODO: Implemet Misplace Detection
    def place(
        self,
        head_id: HeadID,
        carrier_id: LandmarkID,
        device_number: int,
        offsets: list[float, float],
    ) -> bool:

        if (
            carrier_id == LandmarkID.HOMING_FIDUCIAL
            or carrier_id == LandmarkID.BOTTOM_CAMERA
        ):
            self.get_logger().error("Invalid Carrier for Place!")
            return False

        carrier = getattr(self.landmarks, carrier_id.lower())
        run_slots = reshape(carrier.run_slots, 2)
        skew = reshape(carrier.skew, 3)

        output_col, output_row = run_slots[device_number]

        match head_id:
            case HeadID.LEFT:
                actuator = self.pnp.actuator_left
            case HeadID.RIGHT:
                actuator = self.pnp.actuator_right
            case HeadID.CAMERA:
                self.get_logger().error("Can't Place with Camera!")
                return False

        output_x = carrier.x + carrier.x_increment * output_col + offsets[0]
        output_y = carrier.y + carrier.y_increment * output_row + offsets[1]
        output_r = 0.0

        if carrier.markers:
            output_x, output_y, output_r = correct_skew(output_x, output_y, skew)

        self.pnp.move(x=output_x, y=output_y, head=head_id)
        self.pnp.move(r=output_r, jog=True, head=head_id)
        self.pnp.move(z=carrier.z, head=head_id)

        actuator.off()

        self.pnp.move(z=0.0, head=head_id)

        return True
