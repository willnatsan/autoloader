import dataclasses

from .enums import DeviceID, HeadID, RunID, LandmarkID


@dataclasses.dataclass
class HeadPosition:
    x: float = 0.0  # Camera's X-Axis
    y: float = 0.0  # Camera's Y-Axis
    z: float = 30.4  # Tool Head Z-Axis (Midpoint)
    a: float = 0.0  # Left Head Rotation Axis
    b: float = 0.0  # Right Head Rotation Axis

    # Enabling this dataclass to be iterable -> Simplifies a lot of code in main PnP class
    def __iter__(self):
        for field in dataclasses.fields(self):
            yield field.name


@dataclasses.dataclass
class TargetPosition:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    r: float = 0.0


@dataclasses.dataclass
class Offset:
    x: float = 0.0
    y: float = 0.0
    r: float = 0.0


@dataclasses.dataclass
class RunParams:
    head: HeadID = None
    run_type: RunID = RunID.LOADING
    device_type: DeviceID = DeviceID.SMD
    device_count: int = 1
    input_carrier: LandmarkID = None
    output_carrier: LandmarkID = None
