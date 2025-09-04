from .enums import DeviceID

MOTION_NONE = 999
UPDATE_NONE = 999

DEVICE_HEIGHTS = {
    DeviceID.SMD: 1.10,
    DeviceID.TO: 3.66,  # TODO: Get correct value to focus TO Lid
    DeviceID.DIE: 0.00,
}
