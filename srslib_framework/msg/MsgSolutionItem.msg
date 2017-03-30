#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#

# Available solution item actions
uint8 SOLUTION_ITEM_NONE = 0
uint8 SOLUTION_ITEM_MOVE = 1
uint8 SOLUTION_ITEM_ROTATE = 2

uint8 action
srslib_framework/Pose fromPose
srslib_framework/Pose toPose
float32 cost
