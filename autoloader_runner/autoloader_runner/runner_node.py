import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger

from autoloader_data import *
from autoloader_interfaces.msg import Landmarks, MachineStatus, StartStopControl
from autoloader_interfaces.srv import RoutineControl


class RunnerNode(Node):
    def __init__(self) -> None:
        super().__init__("runner_node")

        self.landmarks = Landmarks()
        self.landmarks_updated = False

        self.homed_state = False
        self.run_state = RunState.STOP
        self.run_params = RunParams()

        self.run_thread_active = True
        self.run_thread = threading.Thread(target=self.run)
        self.run_thread.start()

        # Publishers
        self.run_status_pub = self.create_publisher(
            msg_type=UInt32, topic="run_status", qos_profile=10
        )

        # Subscribers
        self.coords_sub = self.create_subscription(
            msg_type=Landmarks,
            topic="coords",
            callback=self.coords_callback,
            qos_profile=10,
        )

        self.machine_status_sub = self.create_subscription(
            msg_type=MachineStatus,
            topic="machine_status",
            callback=self.machine_status_callback,
            qos_profile=10,
        )

        self.start_stop_control_sub = self.create_subscription(
            msg_type=StartStopControl,
            topic="start_stop_control",
            callback=self.start_stop_callback,
            qos_profile=10,
        )

        # Clients
        self.homing_check_cli = self.create_client(
            srv_type=Trigger,
            srv_name="homing_check",
        )

        self.starting_pos_cli = self.create_client(
            srv_type=Trigger,
            srv_name="starting_pos",
        )

        self.routine_cli = self.create_client(
            srv_type=RoutineControl,
            srv_name="routine_control",
        )

    def coords_callback(self, msg: Landmarks) -> None:
        self.landmarks = msg
        self.landmarks_updated = True

    def machine_status_callback(self, msg: MachineStatus) -> None:
        self.homed_state = msg.homed

    def start_stop_callback(self, msg: StartStopControl) -> None:
        self.run_state = msg.command

        if msg.run_params:
            (
                self.run_params.run_type,
                self.run_params.device_type,
            ) = msg.run_params

    def run(self) -> None:
        while self.run_thread_active:
            if self.run_state != RunState.START:
                continue

            starting_pos = self.starting_pos_cli.call(
                Trigger.Request()
            )  # Blocking Call
            if not starting_pos.success:
                self.stop_run(idle=False)
                continue

            match self.run_params.device_type:
                case DeviceID.SMD:
                    self.run_params.head = HeadID.LEFT
                    self.run_params.input_carrier = LandmarkID.SMD_WAFFLE
                    self.run_params.output_carrier = LandmarkID.SMD_CARRIER
                case DeviceID.TO:
                    self.run_params.head = HeadID.RIGHT
                    self.run_params.input_carrier = LandmarkID.TO_WAFFLE
                    self.run_params.output_carrier = LandmarkID.TO_CARRIER
                case DeviceID.DIE:
                    self.get_logger().warning("Can't quite use DIE yet...")
                    self.stop_run()
                    continue

            if self.run_params.run_type == RunID.UNLOADING:
                self.run_params.input_carrier, self.run_params.output_carrier = (
                    self.run_params.output_carrier,
                    self.run_params.input_carrier,
                )

            if not self.homed_state:
                while True:
                    self.routine_cli.call(
                        RoutineControl.Request(routine=RoutineID.HOME)
                    )  # Blocking Call
                    homing_check = self.homing_check_cli.call(
                        Trigger.Request()
                    )  # Blocking Call
                    if homing_check.success:
                        break

            align_machine_response = self.routine_cli.call(
                RoutineControl.Request(routine=RoutineID.ALIGN_MACHINE)
            )  # Blocking Call
            if not align_machine_response.success:
                self.get_logger().error(
                    "Error Occurred when Aligning Machine -> Aborting Run"
                )
                self.stop_run()
                continue

            align_ee_response = self.routine_cli.call(
                RoutineControl.Request(
                    routine=RoutineID.ALIGN_EE,
                    head_curr=HeadID.LEFT,
                )
            )  # Blocking Call
            if not align_ee_response.success:
                self.get_logger().error(
                    "Error Occurred when Aligning EE -> Aborting Run"
                )
                self.stop_run()
                continue

            align_input_carrier_response = self.routine_cli.call(
                RoutineControl.Request(
                    routine=RoutineID.ALIGN_CARRIER,
                    carrier=self.run_params.input_carrier,
                )
            )  # Blocking Call
            if not align_input_carrier_response.success:
                self.get_logger().error(
                    "Error Occurred when Aligning Input Carrier -> Aborting Run"
                )
                self.stop_run()
                continue

            align_output_carrier_response = self.routine_cli.call(
                RoutineControl.Request(
                    routine=RoutineID.ALIGN_CARRIER,
                    carrier=self.run_params.output_carrier,
                )
            )  # Blocking Call
            if not align_output_carrier_response.success:
                self.get_logger().error(
                    "Error Occurred when Aligning Output Carrier -> Aborting Run"
                )
                self.stop_run()
                continue

            if not self.landmarks_updated:
                self.get_logger().error(
                    "Didn't receive updated Landmarks data -> Aborting Run"
                )
                self.stop_run()
                continue

            # Determining Total Number of Devices for Run
            # Note: Could've used Output Carrier here as well since number of Run Slots should be the same
            carrier = getattr(self.landmarks, self.run_params.input_carrier.lower())
            run_slots = reshape(carrier.run_slots, 2)
            total_devices = len(run_slots)

            for device_number in range(total_devices):
                if self.run_state == RunState.PAUSE:
                    continue
                elif self.run_state == RunState.STOP:
                    break

                pick_response = self.routine_cli.call(
                    RoutineControl.Request(
                        routine=RoutineID.PICK,
                        head_curr=self.run_params.head,
                        carrier=self.run_params.input_carrier,
                        device_number=device_number,
                    )
                )  # Blocking Call
                if not pick_response.success:
                    self.get_logger().error(
                        "Failed to Pick Device -> Moving on to Next Device"
                    )
                    continue

                align_device_response = self.routine_cli.call(
                    RoutineControl.Request(
                        routine=RoutineID.ALIGN_DEVICE,
                        head_curr=self.run_params.head,
                        device=self.run_params.device_type,
                    )
                )  # Blocking Call
                if not align_device_response.success:
                    self.get_logger().error(
                        "Failed to Align Device -> Placing Device Back"
                    )
                    self.routine_cli.call(
                        RoutineControl.Request(
                            routine=RoutineID.PLACE,
                            head_curr=self.run_params.head,
                            carrier=self.run_params.output_carrier,
                            device_number=device_number,
                            offsets=[0.0, 0.0],
                        )
                    )
                    continue

                place_response = self.routine_cli.call(
                    RoutineControl.Request(
                        routine=RoutineID.PLACE,
                        head_curr=self.run_params.head,
                        carrier=self.run_params.output_carrier,
                        device_number=device_number,
                        offsets=align_device_response.offsets,
                    )
                )  # Blocking Call
                if not place_response.success:
                    # TODO: Improve Failed Place Routine
                    self.get_logger().error(
                        "Failed to Place Device -> Moving on to Next Device"
                    )
                    continue

            self.stop_run()

    def stop_run(self, idle: bool = True) -> None:
        if idle:
            self.routine_cli.call_async(RoutineControl.Request(routine=RoutineID.IDLE))

        self.run_state = RunState.STOP
        msg = UInt32()
        msg.data = self.run_state
        self.run_status_pub.publish(msg)

    def destroy_node(self) -> None:
        self.run_thread_active = False
        self.run_thread.join()

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    runner_node = RunnerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(runner_node)

    try:
        executor.spin()
    except:
        pass
    finally:
        executor.shutdown()
        runner_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
