import os
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from autoloader_interfaces.msg import (
    Landmark,
    Landmarks,
    MachineStatus,
)
from autoloader_interfaces.srv import UpdateCoords

from .src.constants import UPDATE_NONE
from .src.enums import LandmarkID
from .src.utils import flatten, reshape


def generate_landmark(landmark: LandmarkID) -> Landmark:
    with open(os.path.dirname(__file__) + "/src/coords.yaml", "r") as coords_file:
        coords = yaml.safe_load(coords_file)
        msg = Landmark()

        msg.id = landmark

        # Values may change at runtime -> Persistent
        msg.x = coords[landmark]["x"]
        msg.y = coords[landmark]["y"]
        msg.z = coords[landmark]["z"]

        if landmark in (
            LandmarkID.SMD_CARRIER,
            LandmarkID.SMD_WAFFLE,
            LandmarkID.TO_CARRIER,
            LandmarkID.TO_WAFFLE,
        ):
            # Values will not change at runtime -> Persistent
            msg.x_increment = coords[landmark]["x_increment"]
            msg.y_increment = coords[landmark]["y_increment"]
            msg.available_slots = flatten(coords[landmark]["slots"])
            msg.markers = flatten(coords[landmark]["markers"])
            msg.marker_radius = coords[landmark]["marker_radius"]

            # Values will change at runtime -> Persistent
            msg.run_slots = flatten(coords[landmark]["run_slots"])
            if coords[landmark]["failed_slots"]:
                msg.failed_slots = flatten(coords[landmark]["failed_slots"])
            else:
                msg.failed_slots = []

            # Values only evaluated at runtime -> Not Persistent
            msg.skew = flatten([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # Default: No Skew

        return msg


def generate_coords() -> Landmarks:
    msg = Landmarks()

    for landmark_id in (
        LandmarkID.HOMING_FIDUCIAL,
        LandmarkID.BOTTOM_CAMERA,
        LandmarkID.SMD_WAFFLE,
        LandmarkID.SMD_CARRIER,
        LandmarkID.TO_WAFFLE,
        LandmarkID.TO_CARRIER,
    ):
        landmark_populated = generate_landmark(landmark_id)
        setattr(msg, landmark_id.lower(), landmark_populated)

    return msg


class CoordsNode(Node):
    def __init__(self) -> None:
        super().__init__("coords_node")

        self.homed_state = False
        self.landmarks = generate_coords()
        self.update_lock = threading.Lock()

        # Publishers
        self.coords_pub = self.create_publisher(
            msg_type=Landmarks,
            topic="coords",
            qos_profile=10,
        )
        self.coords_timer = self.create_timer(
            timer_period_sec=1 / 12,
            callback=self.publish_coords,
        )

        # Subscribers
        self.machine_status_sub = self.create_subscription(
            msg_type=MachineStatus,
            topic="machine_status",
            callback=self.machine_status_callback,
            qos_profile=10,
        )

        # Services
        self.update_coords_srv = self.create_service(
            srv_type=UpdateCoords,
            srv_name="update_coords",
            callback=self.update_coords_local,
        )

    def publish_coords(self) -> None:
        with self.update_lock:
            self.coords_pub.publish(self.landmarks)

    def machine_status_callback(self, msg: MachineStatus) -> None:
        self.homed_state = msg.homed

    def update_coords_local(
        self, request: UpdateCoords.Request, response: UpdateCoords.Response
    ) -> UpdateCoords.Response:

        with self.update_lock:
            landmark = getattr(self.landmarks, request.landmark_id.lower())

            if self.homed_state:
                if request.x != UPDATE_NONE:
                    landmark.x = round(request.x, 2)
                if request.y != UPDATE_NONE:
                    landmark.y = round(request.y, 2)
                if request.z != UPDATE_NONE:
                    landmark.z = round(request.z, 2)
                response.success = True
            elif not self.homed_state and (
                request.x != UPDATE_NONE
                or request.y != UPDATE_NONE
                or request.z != UPDATE_NONE
            ):
                self.get_logger().error(
                    "Unable to update XYZ coords when Machine not Homed"
                )
                response.success = False

            # `run_slots`, `failed_slots`, and `skew` only valid for Carrier Landmarks
            if (
                request.landmark_id != LandmarkID.HOMING_FIDUCIAL
                and request.landmark_id != LandmarkID.BOTTOM_CAMERA
            ):
                if request.run_slots:
                    landmark.run_slots = request.run_slots
                if request.failed_slots != [UPDATE_NONE, UPDATE_NONE]:
                    landmark.failed_slots = request.failed_slots
                if request.skew:
                    landmark.skew = request.skew
                response.success = True
            elif (
                request.landmark_id == LandmarkID.HOMING_FIDUCIAL
                or request.landmark_id == LandmarkID.BOTTOM_CAMERA
            ) and (request.run_slots or request.failed_slots or request.skew):
                self.get_logger().error(
                    "Unable to update Run Slots, Failed Slots, or Skew for Non-Carrier Landmarks"
                )
                response.success = False

            return response

    def update_coords_file(self) -> None:
        with open(
            os.path.dirname(__file__) + "/src/coords.yaml", "r"
        ) as coords_file_read:
            coords = yaml.safe_load(coords_file_read)

            for landmark_id in (
                LandmarkID.HOMING_FIDUCIAL,
                LandmarkID.BOTTOM_CAMERA,
                LandmarkID.SMD_WAFFLE,
                LandmarkID.SMD_CARRIER,
                LandmarkID.TO_WAFFLE,
                LandmarkID.TO_CARRIER,
            ):
                landmark = getattr(self.landmarks, landmark_id.lower())

                # These are the only values that may change at runtime that we want to persist
                # `skew` also changes at runtime, but should reset with every new execution!
                coords[landmark_id]["x"] = landmark.x
                coords[landmark_id]["y"] = landmark.y
                coords[landmark_id]["z"] = landmark.z

                if (
                    landmark_id != LandmarkID.HOMING_FIDUCIAL
                    and landmark_id != LandmarkID.BOTTOM_CAMERA
                ):
                    coords[landmark_id]["run_slots"] = reshape(landmark.run_slots, 2)
                    if landmark.failed_slots:
                        coords[landmark_id]["failed_slots"] = reshape(
                            landmark.failed_slots, 2
                        )
                    else:
                        coords[landmark_id]["failed_slots"] = None

        with open(
            os.path.dirname(__file__) + "/src/coords.yaml", "w"
        ) as coords_file_write:
            yaml.dump(coords, coords_file_write)

    def destroy_node(self) -> None:
        # Dump persistent data before closing
        self.update_coords_file()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    coords_node = CoordsNode()
    executor = SingleThreadedExecutor()
    executor.add_node(coords_node)

    try:
        executor.spin()
    except:
        pass
    finally:
        executor.shutdown()
        coords_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
