from rclpy.node import Node

from autoloader_data import *
from autoloader_interfaces.msg import MachineStatus
from autoloader_interfaces.srv import ActuatorControl, MotionControl

from .src._pnp import PnP


class DriverNode(Node):
    def __init__(self, pnp_machine: PnP) -> None:
        super().__init__("driver_node")

        self.pnp = pnp_machine

        # Publishers
        self.machine_status_pub = self.create_publisher(
            msg_type=MachineStatus,
            topic="machine_status",
            qos_profile=10,
        )
        self.machine_status_timer = self.create_timer(
            timer_period_sec=1 / 12,
            callback=self.publish_machine_status,
        )

        # Services
        self.actuator_srv = self.create_service(
            srv_type=ActuatorControl,
            srv_name="actuator_control",
            callback=self.actuator_callback,
        )

        self.motion_srv = self.create_service(
            srv_type=MotionControl,
            srv_name="motion_control",
            callback=self.motion_callback,
        )

    """
    Publisher Callbacks
    """

    def publish_machine_status(self) -> None:
        msg = MachineStatus()
        msg.homed = self.pnp.homed
        msg.position_absolute = [
            self.pnp.position_curr_absolute.x,
            self.pnp.position_curr_absolute.y,
            self.pnp.position_curr_absolute.z,
        ]
        msg.position_relative = [
            self.pnp.position_curr_relative.x,
            self.pnp.position_curr_relative.y,
            self.pnp.position_curr_relative.z,
        ]

        self.machine_status_pub.publish(msg)

    """
    Service Callbacks
    """

    def actuator_callback(
        self, request: ActuatorControl.Request, response: ActuatorControl.Response
    ) -> ActuatorControl.Response:

        actuator_id = ActuatorID(request.actuator)

        if request.active:
            response.success = self.pnp.actuators[actuator_id].on()
        else:
            response.success = self.pnp.actuators[actuator_id].off()

        return response

    def motion_callback(
        self, request: MotionControl.Request, response: MotionControl.Response
    ) -> MotionControl.Response:

        move_type = MoveID(request.move_type)
        head_curr = HeadID(request.head_curr)

        match move_type:
            case MoveID.HOME:
                response.success = self.pnp.home()
            case MoveID.ABSOLUTE:
                response.success = self.pnp.move(
                    x=request.x,
                    y=request.y,
                    z=request.z,
                    r=request.r,
                    jog=False,
                    head=head_curr,
                )
            case MoveID.JOG:
                response.success = self.pnp.move(
                    x=request.x,
                    y=request.y,
                    z=request.z,
                    r=request.r,
                    jog=True,
                    head=head_curr,
                )

        return response
