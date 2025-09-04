import dataclasses
import logging
import math
import re
import time

from autoloader_data import (
    HeadID,
    PumpID,
    GripperID,
    LightID,
    Offset,
)

from .comms import SerialManager

logger = logging.getLogger(__name__)


@dataclasses.dataclass
class Light:
    id: LightID
    serial_manager: SerialManager

    def on(self, r: int = 255, g: int = 255, b: int = 255, a: int = 255) -> bool:
        # 'M150' command -> "Set RGB Colour" command
        if not self.serial_manager.send(f"M150 P{a} R{r} U{g} B{b} S{self.id}"):
            return False

        time.sleep(0.5)

        return True

    def off(self) -> bool:
        return self.serial_manager.send(f"M150 P0 R0 U0 B0 S{self.id}")


@dataclasses.dataclass
class Pump:
    id: PumpID
    serial_manager: SerialManager
    pressure_range: tuple[int, int]

    def on(self) -> bool:
        # P{id} refers to the target Pump itself
        # P{id+1} refers to the target Valve

        # 'M106' command -> "Set Fan Speed" command (Pump & Valve are treated as Fans)
        pump_on_commands = [
            f"M106 P{self.id} S255",
            f"M106 P{self.id + 1} S255",
        ]

        for command in pump_on_commands:
            if not self.serial_manager.send(command):
                return False

        time.sleep(0.75)

        return True

    def off(self) -> bool:
        # P{id} refers to the target Pump itself
        # P{id+1} refers to the target Valve

        # 'M107' command -> "Turn Fan Off" command (Pump & Valve are treated as Fans)
        pump_off_commands = [f"M107 P{self.id}", f"M107 P{self.id + 1}"]

        for command in pump_off_commands:
            if not self.serial_manager.send(command):
                return False

        time.sleep(0.75)

        return True

    def read(self, timeout: int = 1) -> int:
        # 'M260' command -> "I2C Send" command
        # Select correct Pump via Multiplexer
        match self.id:
            case PumpID.PUMP_LEFT:
                if not self.serial_manager.send(f"M260 A112 B1 S1"):
                    return -1
            case PumpID.PUMP_RIGHT:
                if not self.serial_manager.send(f"M260 A112 B2 S1"):
                    return -1

        # Pressure Reading Setup
        pump_read_setup_commands = [
            "M260 A109",
            "M260 B48",
            "M260 B27",
            "M260 S1",
        ]
        for command in pump_read_setup_commands:
            if not self.serial_manager.send(command):
                return -1

        time.sleep(0.03)

        # 'M260' command -> "I2C Send" command
        # 'M261' command -> "I2C Request" command
        # Read 0x06 address (Most Significant Bit) for Pressure Reading
        pump_read_commands = [
            "M260 A109 B6 S1",
            "M261 A109 B1 S2",
        ]
        for command in pump_read_commands:
            if not self.serial_manager.send(command):
                return -1

        start_time = time.perf_counter()
        while time.perf_counter() - start_time < timeout:
            response = self.serial_manager.read_line()
            if response == "FAILED":
                continue

            data = re.search("data:(...)", response)
            if data:
                return int(data.group(1))

        return -1

    def part_detected(self) -> bool:
        pressure = self.read()

        return 0 < pressure < sum(self.pressure_range) // 2


@dataclasses.dataclass
class Gripper:
    id: GripperID
    serial_manager: SerialManager
    on_angle: int
    off_angle: int

    def on(self) -> bool:
        # P{id} refers to the target Servo
        # S{val} refers to the target Servo Angle

        # 'M280' command -> "Set Servo Position"
        if not self.serial_manager.send(f"M280 P{self.id} S{self.on_angle}"):
            return False

        time.sleep(0.5)  # TODO: Find exact settling time needed

        return True

    def off(self) -> bool:
        # P{id} refers to the target Servo
        # S{val} refers to the target Servo Angle

        # 'M280' command -> "Set Servo Position"
        if not self.serial_manager.send(f"M280 P{self.id} S{self.off_angle}"):
            return False

        time.sleep(0.5)  # TODO: Find exact settling time needed

        return True


@dataclasses.dataclass
class HeadEE:
    id: HeadID
    end_effector: Pump | Gripper
    axis: str
    runout: float = 0.0
    phasing: float = 0.0
    rms_error: float = 0.0

    def get_offset(self, rotation_angle: float) -> Offset:
        offset = Offset()

        offset.x = self.runout * math.cos(math.radians(rotation_angle - self.phasing))
        offset.y = self.runout * math.sin(math.radians(rotation_angle - self.phasing))

        return offset
