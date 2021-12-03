Header header
#Chosen angle from LD result (degree). Positive value means right turn
float64 ld_angle
#Desired angle of the steering wheel (hand wheel). Positive value means counter-clockwise turn
float64 steering_angle
#1:Available, 2:Attention, 4:Unavailable. See LDW_Types.h
uint8 status
float64 curr_curvature
float64 desired_curvature
float64 look_ahead_distance
#1: straight, 2: left Curve 3: right Curve
uint8 curveStatus
