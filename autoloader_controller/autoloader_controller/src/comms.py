import logging
import time

import serial
import serial.tools.list_ports

logger = logging.getLogger(__name__)

logging.basicConfig(
    format="[{asctime}] {levelname} : {name} : {message}",
    style="{",
    datefmt="%Y/%m/%d %H:%M",
    level=logging.INFO,
)


class SerialManager:
    def __init__(self) -> None:
        self._serial = serial.Serial()
        self._serial.baudrate = 119200
        self._serial.timeout = 1

    def open(self) -> bool:
        device_id = "0483:5740"
        for port, _, hardware_id in serial.tools.list_ports.comports():
            if device_id in hardware_id:
                self._serial.port = port
                break
        if not self._serial.port:
            logger.error("Device not detected on any port")
            return False

        if self._serial.is_open:
            logger.warning("Serial Port is already open")
            return False

        self._serial.open()
        if not self._serial.is_open:
            logger.error("Unable to open Serial Port")
            return False
        self._serial.reset_input_buffer()

        return True

    def close(self) -> bool:
        self._serial.close()

        if self._serial.is_open:
            logger.error("Failed to close Serial Port")
            return False

        return True

    def clear_queue(self, timeout: int) -> bool:
        # 'M400' command -> "Finish Moves" command
        # 'M118' command -> "Serial Print" command
        for command in ("M400", "M118 E1 done"):
            response = self.send_response(command)
            if "echo:done" in response:
                return True

        # Waiting to receive 'echo:done' to confirm "Finish Moves" completion
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < timeout:
            response = self.read_line()
            if "echo:done" in response:
                return True

        logger.warning(
            "Failed to receive 'echo:done' response from device within timeout"
        )
        return False

    def clear_input_buffer(self):
        self._serial.reset_input_buffer()
        while self._serial.in_waiting > 0:
            _ = self._serial.readline()

    def send(self, gcode: str) -> bool:
        if self._serial.is_open:
            self.clear_input_buffer()
            if self._serial.write(gcode.encode("utf-8") + b"\n") is None:
                logger.error(f"Unable to send {gcode} command; Failed to write")
                return False
            return True

        logger.error(f"Unable to send {gcode} command, Serial Port not open")
        return False

    def send_response(self, gcode: str) -> str:
        try:
            if self.send(gcode):
                response = self._serial.readline().decode("utf-8")
                return response
        except:
            return "FAILED"

    def read_line(self) -> str:
        try:
            response = self._serial.readline().decode("utf-8")
            return response
        except:
            return "FAILED"
