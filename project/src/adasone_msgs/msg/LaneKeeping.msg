Header header
bool result                  # False: No result from Lane Detector
uint8 max_index              # Max index of angle array
float64[20] angle            # Lane keeping angles in every 5m distances (5, 10, 15...)
int64 lat_offset_center      # Lateral offset compared to center of lane in centimeter	
#float64 curvature            # Cuvature of lane = 1/curvature Radius
uint8 curve_stt
float64 curve_value
int64 lane_width             # Estimated lane width in centimeter. Return 0 when lane is not fully detected
