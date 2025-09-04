import enum


class HeadID(enum.IntEnum):
    LEFT = 0
    RIGHT = 1
    CAMERA = 2


class CameraID(enum.IntEnum):
    TOP_CAMERA = 0
    BOT_CAMERA = 1


class ActuatorID(enum.IntEnum):
    TOP_LIGHT = 0
    BOT_LIGHT = 1
    LEFT_ACTUATOR = 2
    RIGHT_ACTUATOR = 3


class PumpID(enum.IntEnum):
    PUMP_LEFT = 0  # Default: Pump on Left Head
    PUMP_RIGHT = 2


class GripperID(enum.IntEnum):
    GRIPPER_LEFT = 0
    GRIPPER_RIGHT = 1  # Default: Gripper on Right Head


class LightID(enum.IntEnum):
    BOT_LIGHT = 0
    TOP_LIGHT = 1


class MoveID(enum.IntEnum):
    HOME = 0
    ABSOLUTE = 1
    JOG = 2


class LandmarkID(str, enum.Enum):
    HOMING_FIDUCIAL = "HOMING_FIDUCIAL"
    BOTTOM_CAMERA = "BOTTOM_CAMERA"
    SMD_WAFFLE = "SMD_WAFFLE"
    SMD_CARRIER = "SMD_CARRIER"
    TO_WAFFLE = "TO_WAFFLE"
    TO_CARRIER = "TO_CARRIER"


class DeviceID(enum.IntEnum):
    SMD = 0
    TO = 1
    DIE = 2


class RoutineID(enum.IntEnum):
    HOME = 0
    IDLE = 1
    PARK_Z = 2
    ALIGN_MACHINE = 3
    ALIGN_EE = 4
    ALIGN_DEVICE = 5
    ALIGN_CARRIER = 6
    PICK = 7
    PLACE = 8


class RunID(enum.IntEnum):
    LOADING = 0
    UNLOADING = 1


class RunState(enum.IntEnum):
    START = 0
    PAUSE = 1
    STOP = 2


class PositionCommand(enum.IntEnum):
    MOVE = 0
    CAPTURE = 1


class JogDirection(enum.IntEnum):
    BACK = 0
    LEFT = 1
    RIGHT = 2
    FORWARD = 3
    UP = 4
    DOWN = 5
    CLOCKWISE = 6
    COUNTER_CLOCKWISE = 7


class PopUpID(str, enum.Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
