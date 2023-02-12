# TargetForce : The target force to be reached when gripping and holding a workpiece.
#        It must be provided in 1/10th Newtons.
#        The valid range is 0 to 400 for the RG2-FT

uint16 TargetForce

# TargetWidth : The target width between the finger to be moved to and maintained.
#        It must be provided in 1/10th millimeters.
#        The valid range is 0 to 1000
#        Please note that the target width should be provided corrected for any fingertip offset,

uint16 TargetWidth

# Control : The control field is used to start and stop gripper motion.
#        Only one option should be set at a time.
#        Please note that the gripper will not start a new motion
#        before the one currently being executed is done (see busy flag in the Status field).
# 0x0001 - grip
#          Start the motion, with the preset target force and width.
#          Width is calculated without the fingertip offset.
#          Please note that the gripper will ignore this command
#          if the busy flag is set in the status field.
# 0x0000 - stop
#          Stop the current motion.

uint8 Control