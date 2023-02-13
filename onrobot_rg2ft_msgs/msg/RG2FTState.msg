uint8 StatusL # Reads low (0x0000) when there is no error with the left finger sensor.
int16 FxL # Left finger sensor's force value along the X axis (in the sensor coordinate system) in 1/10N.
int16 FyL # Left finger sensor's force value along the Y axis (in the sensor coordinate system) in 1/10N.
int16 FzL # Left finger sensor's force value along the Z axis (in the sensor coordinate system) in 1/10N.
int16 TxL # Left finger sensor's torque value about the X axis (in the sensor coordinate system) in 1/100 Nm.
int16 TyL # Left finger sensor's torque value about the Y axis (in the sensor coordinate system) in 1/100 Nm.
int16 TzL # Left finger sensor's torque value about the Z axis (in the sensor coordinate system) in 1/100 Nm.
uint8 StatusR # Same as the left above.
int16 FxR # Same as the left above.
int16 FyR # Same as the left above.
int16 FzR # Same as the left above.
int16 TxR # Same as the left above.
int16 TyR # Same as the left above.
int16 TzR # Same as the left above.
uint8 ProximityStatusL # Reads low (0x0000) when there is no error with the left proximity sensor.
int16 ProximityValueL # Reads the current distance from the left proximity sensor in 1/10 mm.
uint8 ProximityStatusR # Same as the left above.
int16 ProximityValueR # Same as the left above.
int16 ActualGripperWidth # Indicates the current width between the gripper fingers in 1/10 millimeters.
uint8 GripperBusy # High (1) when a motion is ongoing, low (0) when not. The gripper will only accept new commands when this flag is low.
uint8 GripDetected # High (1) when an internal or external grip is detected.