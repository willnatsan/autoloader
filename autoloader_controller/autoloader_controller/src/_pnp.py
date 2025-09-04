import logging
import os

from autoloader_data import *

from .actuators import *
from .comms import *
from .vision import *
from .utils import *

logger = logging.getLogger(__name__)


class PnP:
    def __init__(self) -> None:
        # Private Attributes
        self._serial_manager = SerialManager()

        self._position_next = HeadPosition()
        self._position_z_midpoint = 30.4

        self._normal_feedrate = 25000
        self._backlash_feedrate = 0.1 * self._normal_feedrate
        self._backlash_offset = -0.01

        self._max_x = 635
        self._max_y = 485
        self._max_z = self._position_z_midpoint * 2

        # Public Attributes
        self.head_offsets = [
            Offset(x=23.40, y=64.35),  # Left Head Offset
            Offset(x=-24.10, y=64.75),  # Right Head Offset
            Offset(x=0.0, y=0.0),  # Camera Offset
        ]
        self.head_selected = HeadID.LEFT
        self.position_curr_absolute = HeadPosition()
        self.position_curr_relative = HeadPosition()

        self.homed = False
        self.connected = False

        # Cameras
        self.camera_top = Camera(
            src="/dev/top_camera",
            units_per_pixel_x=0.015873015873015872,
            units_per_pixel_y=0.015873015873015872,
            units_scale_factor=1,  # TODO: Get actual value (3D Unites per Pixel)
        )
        self.camera_bot = Camera(
            src="/dev/bottom_camera",
            units_per_pixel_x=0.03571428571428571,
            units_per_pixel_y=-0.03571428571428571,
        )

        # Actuators
        self.light_top = Light(
            id=LightID.TOP_LIGHT, serial_manager=self._serial_manager
        )
        self.light_bot = Light(
            id=LightID.BOT_LIGHT, serial_manager=self._serial_manager
        )

        self.actuator_left = Pump(
            id=PumpID.PUMP_LEFT,
            serial_manager=self._serial_manager,
            pressure_range=(228, 245),
        )
        self.actuator_right = Gripper(
            id=GripperID.GRIPPER_RIGHT,
            serial_manager=self._serial_manager,
            on_angle=130,
            off_angle=155,
        )

        self.actuators = {
            ActuatorID.TOP_LIGHT: self.light_top,
            ActuatorID.BOT_LIGHT: self.light_bot,
            ActuatorID.LEFT_ACTUATOR: self.actuator_left,
            ActuatorID.RIGHT_ACTUATOR: self.actuator_right,
        }

        self.head_left = HeadEE(
            id=HeadID.LEFT, end_effector=self.actuator_left, axis="a"
        )
        self.head_right = HeadEE(
            id=HeadID.RIGHT, end_effector=self.actuator_right, axis="b"
        )

    def connect(self) -> bool:
        if not self._serial_manager.open():
            return False

        # 'G90' -> "Set all Axes to Absolute"
        # 'M260' -> "Send to I2C Address"
        # 'G0 F' -> "Set Linear Move Speed"
        # 'M204 T' -> "Set Travel Acceleration"
        # 'M914' -> "Set Sensorless Homing Sensitivity"
        # 'M906' -> "Set Motor Current Limit"
        # 'M92' -> "Set Steps per Unit"
        boot_commands = [
            "G90",
            "M260 A112 B1 S1",
            "M260 A109",
            "M260 B48",
            "M260 B27",
            "M260 S1",
            "M260 A112 B2 S1",
            "M260 A109",
            "M260 B48",
            "M260 B27",
            "M260 S1",
            "M204 T1000",
            "M914 X120 Y120",
            "M906 X1200 Y1200",
            "M92 X319.744345 Y319.233918",
        ]
        for command in boot_commands:
            if not self._serial_manager.send(command):
                return False

        self.steppers_on()
        self.actuator_right.off()

        self.connected = True

        return True

    def disconnect(self) -> bool:

        # Disable Actuators
        if not (self.actuator_left.off() and self.actuator_right.off()):
            return False
        if not (self.light_top.off() and self.light_bot.off()):
            return False

        # Disengage Steppers
        if not self.steppers_off():
            return False

        if not self._serial_manager.close():
            return False

        self.homed = False
        self.connected = False

        return True

    """
    Motion Control
    """

    def set_speed(self, speed: int) -> bool:
        return self._serial_manager.send(f"G0 F{speed}")

    def steppers_on(self) -> bool:
        # "M17" -> 'Enable Steppers'
        return self._serial_manager.send("M17")

    def steppers_off(self) -> bool:
        # "M18" -> 'Disable Steppers'
        return self._serial_manager.send("M18")

    def home(self) -> bool:
        # 'G28' -> "Auto Home"
        # 'G92' -> "Set Machine Coordinates" -> Just needed for Homing
        homing_commands = [
            "G28",
            "G92 X0 Y0 A0 B0",
            "G1 A10 B10 F50000",
            "G1 A0 B0 F50000",
        ]

        for command in homing_commands:
            if not self._serial_manager.send(command):
                return False

        self.finish_move(timeout=20)

        for axis in self.position_curr_absolute:
            if axis == "z":
                setattr(self.position_curr_absolute, axis, self._position_z_midpoint)
            else:
                setattr(self.position_curr_absolute, axis, 0.0)

        self.homed = True

        return True

    def move(
        self,
        x: float = MOTION_NONE,
        y: float = MOTION_NONE,
        z: float = MOTION_NONE,
        r: float = MOTION_NONE,
        jog: bool = False,
        head: HeadID = HeadID.CAMERA,
    ):
        if not self.homed:
            logger.warning("PnP not Homed; Unable to move")
            return False

        if x != MOTION_NONE:
            if not jog:
                self._position_next.x = x + self.head_offsets[head].x
            else:
                self._position_next.x = x + self.position_curr_absolute.x

        if y != MOTION_NONE:
            if not jog:
                self._position_next.y = y + self.head_offsets[head].y
            else:
                self._position_next.y = y + self.position_curr_absolute.y

        if z != MOTION_NONE:
            match head:
                case HeadID.LEFT:
                    if not jog:
                        self._position_next.z = self._position_z_midpoint - z
                    else:
                        self._position_next.z = -z + self.position_curr_absolute.z
                case HeadID.RIGHT:
                    if not jog:
                        self._position_next.z = self._position_z_midpoint + z
                    else:
                        self._position_next.z = z + self.position_curr_absolute.z
                case HeadID.CAMERA:
                    self._position_next.z = self._position_z_midpoint

        if r != MOTION_NONE:
            match head:
                case HeadID.LEFT:
                    if not jog:
                        self._position_next.a = r
                    else:
                        self._position_next.a = r + self.position_curr_absolute.a
                case HeadID.RIGHT:
                    if not jog and -180 < r < 90:
                        self._position_next.b = r
                    elif jog and -180 < r + self.position_curr_absolute.b < 90:
                        self._position_next.b = r + self.position_curr_absolute.b
                    else:
                        logger.error("Gripper Rotation Limited to -180 to 90 Degrees!")
                case HeadID.CAMERA:
                    self._position_next.a = 0.0
                    self._position_next.b = 0.0

        # Runout Compensation
        if (head == HeadID.LEFT or head == HeadID.RIGHT) and r is not None:
            runout_offset = (self.head_left, self.head_right)[head].get_offset(r)
            self._position_next.x += runout_offset.x
            self._position_next.y += runout_offset.y

        if not self.check_move(self._position_next):
            logger.warning("Invalid Move Requested")
            return False

        # Backlash Compensation for XY -> 'OneSidedPositioning'
        # https://github.com/openpnp/openpnp/wiki/Backlash-Compensation
        command_initial = "G1"
        for axis in self._position_next:
            value_new = getattr(self._position_next, axis)
            value_curr = getattr(self.position_curr_absolute, axis)

            if value_new == value_curr:
                continue

            if axis == "x" or axis == "y":
                command_initial += f" {axis.upper()}{value_new + self._backlash_offset}"
            else:
                command_initial += f" {axis.upper()}{value_new}"

        if command_initial != "G1":
            command_initial += f" F{self._normal_feedrate}"
            if not self._serial_manager.send(command_initial):
                return False

        self.finish_move()

        # Moving to Actual Target Position
        command_final = "G1"
        for axis in self._position_next:
            value_new = getattr(self._position_next, axis)

            command_final += f" {axis.upper()}{value_new}"
            setattr(self.position_curr_absolute, axis, value_new)

            offset = getattr(self.head_offsets[self.head_selected], axis, 0.0)
            setattr(self.position_curr_relative, axis, value_new - offset)

        if command_final != "G1":
            command_final += f" F{self._backlash_feedrate}"
            if not self._serial_manager.send(command_final):
                return False

        self.finish_move()

        return True

    def check_move(self, head_position: HeadPosition) -> bool:
        return (
            (0 <= head_position.x + self._backlash_offset < self._max_x)
            and (0 <= head_position.y + self._backlash_offset < self._max_y)
            and (0 <= head_position.z < self._max_z)
        )

    def finish_move(self, timeout: int = 3) -> bool:
        return self._serial_manager.clear_queue(timeout)

    def reset_position(self, head_position: HeadPosition) -> bool:
        # 'G92' -> "Set Machine Coordinates"
        if not self._serial_manager.send(f"G92 X{head_position.x} Y{head_position.y}"):
            return False

        self.position_curr_absolute.x = head_position.x
        self.position_curr_absolute.y = head_position.y

        self._position_next.x = head_position.x
        self._position_next.y = head_position.y

        self.position_curr_relative.x = (
            head_position.x - self.head_offsets[self.head_selected].x
        )
        self.position_curr_relative.y = (
            head_position.y - self.head_offsets[self.head_selected].y
        )

        return True

    """
    Offset, & Orientation Detection
    """

    def get_device_orientation(self, device_id: DeviceID) -> tuple[bool, int]:
        rotation = 0

        # Camera settling routine (Simple delay)
        time.sleep(0.25)

        result, image = self.camera_bot.get_frame()
        if not result:
            logger.error("Unable to get frame from Bottom Camera")
            return False, rotation

        match device_id:
            case DeviceID.SMD:
                device_prefix = "smd_"

            case DeviceID.TO:
                logger.warning("TO Orientation Correction -> Unimplemented")
                return False, rotation

            case DeviceID.DIE:
                logger.warning("DIE Orientation Correction -> Unimplemented")
                return False, rotation

        image_cropped = crop_image(image, 75)

        # Template Matching
        # https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html
        max_val_curr = 0
        for angle in (-180, -90, 0, 90, 180):
            template = cv2.imread(
                os.path.dirname(__file__) + f"/templates/{device_prefix}{angle}.png"
            )
            result = cv2.matchTemplate(image_cropped, template, cv2.TM_CCOEFF_NORMED)
            max_val = cv2.minMaxLoc(result)[1]

            if max_val > max_val_curr:
                max_val_curr = max_val
                rotation = angle

        return True, rotation

    def get_device_offset(self, device_id: DeviceID) -> tuple[bool, Offset]:
        offset = Offset()

        # Camera settling routine (Simple delay)
        time.sleep(0.25)

        result, image = self.camera_bot.get_frame()
        if not result:
            logger.error("Unable to get frame from Bottom Camera")
            return False, offset

        match device_id:
            case DeviceID.SMD:
                image_masked = mask_image(image, 150)

                result, rect_coords = find_rectangular(image_masked)
                if not result:
                    return False, offset

                rect_x, rect_y, rect_r = rect_coords
                offset.x, offset.y = self.camera_bot.get_mm_offset(rect_x, rect_y)
                offset.r = rect_r

            case DeviceID.TO:
                # TODO: Get correct value for TO Lid Radius
                image_masked = mask_image(image, 80)
                result, circle_coords = find_circular(image_masked, 51)
                if not result:
                    return False, offset

                circle_x, circle_y = circle_coords
                offset.x, offset.y = self.camera_bot.get_mm_offset(circle_x, circle_y)

            case DeviceID.DIE:
                logger.warning("Can't quite do DIE yet...")
                pass

        return True, offset

    def get_fiducial_offset(self, radius: int = 29) -> tuple[bool, Offset]:
        offset = Offset()

        # Camera settling routine (Simple delay)
        time.sleep(0.25)

        result, image = self.camera_top.get_frame()
        if not result:
            logger.error("Unable to get frame from Top Camera")
            return False, offset

        image_masked = mask_image(image, 300)
        result, circle_coords = find_circular(image_masked, radius=radius)
        if not result:
            return False, offset

        circle_x, circle_y = circle_coords
        offset.x, offset.y = self.camera_top.get_mm_offset(circle_x, circle_y)

        return True, offset

    def get_ee_offset(self, head_curr: HeadID) -> tuple[bool, Offset]:
        offset = Offset()

        # Camera settling routine (Simple delay)
        time.sleep(0.5)

        result, image = self.camera_bot.get_frame()
        if not result:
            logger.error("Unable to get frame from Bottom Camera")
            return False, offset

        image_masked = mask_image(image, 200)

        match head_curr:
            case HeadID.LEFT:
                result, circle_coords = find_circular(image_masked, 10)
                if not result:
                    return False, offset

            case HeadID.RIGHT:
                # TODO: Get actual value for Gripper Mark Radius
                result, circle_coords = find_circular(image_masked, 51)
                if not result:
                    return False, offset

            case HeadID.CAMERA:
                self.get_logger().warning("Unable to get offset from Camera Head")
                return False, offset

        circle_x, circle_y = circle_coords
        offset.x, offset.y = self.camera_bot.get_mm_offset(circle_x, circle_y)

        return True, offset
