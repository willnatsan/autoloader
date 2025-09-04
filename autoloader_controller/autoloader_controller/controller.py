import logging

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .src._pnp import PnP
from .driver_node import DriverNode
from .routine_node import RoutineNode
from .vision_node import VisionNode

logger = logging.getLogger(__name__)


def main(args=None):
    rclpy.init(args=args)
    pnp_machine = PnP()
    pnp_machine.connect()

    driver_node = DriverNode(pnp_machine)
    routine_node = RoutineNode(pnp_machine)
    vision_node = VisionNode(pnp_machine)

    executor = MultiThreadedExecutor()
    executor.add_node(driver_node)
    executor.add_node(routine_node)
    executor.add_node(vision_node)

    try:
        executor.spin()
    except:
        pass
    finally:
        pnp_machine.disconnect()
        executor.shutdown()
        driver_node.destroy_node()
        routine_node.destroy_node()
        vision_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
