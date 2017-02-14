#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#

uint32 upTime

# Motion Status
bool frontEStop
bool backEStop
bool wirelessEStop
bool bumpSensor
bool pause
bool hardStop

# Failure Status
bool safetyProcessorFailure
bool brainstemFailure
bool brainstemTimeoutFailure
bool brainTimeoutFailure
bool rightMotorFailure
bool leftMotorFailure
bool rpmMessageTimeout
bool velocityViolation