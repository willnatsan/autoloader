from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from autoloader_data import *

from .src._pnp import PnP


class VisionNode(Node):
    def __init__(self, pnp_machine: PnP) -> None:
        super().__init__("vision_node")

        self.pnp = pnp_machine
        self.image_bridge = CvBridge()

        # Publishers
        self.top_cam_pub = self.create_publisher(
            msg_type=Image,
            topic="top_camera",
            qos_profile=10,
        )
        self.top_cam_timer = self.create_timer(
            timer_period_sec=1 / 12,
            callback=self.publish_top_frame,
        )

        self.bot_cam_pub = self.create_publisher(
            msg_type=Image,
            topic="bot_camera",
            qos_profile=10,
        )
        self.bot_cam_timer = self.create_timer(
            timer_period_sec=1 / 12,
            callback=self.publish_bot_frame,
        )

    def publish_top_frame(self) -> None:
        ret, top_frame = self.pnp.camera_top.get_frame()
        if not ret:
            return

        msg = self.image_bridge.cv2_to_imgmsg(top_frame)

        self.top_cam_pub.publish(msg)

    def publish_bot_frame(self) -> None:
        ret, bot_frame = self.pnp.camera_bot.get_frame()
        if not ret:
            return

        msg = self.image_bridge.cv2_to_imgmsg(bot_frame)

        self.bot_cam_pub.publish(msg)
