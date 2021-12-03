Header header

float64 imu_time               # imu ros time
uint32 iTOW                    # GPS Millisecond time of week [ms]
int8 is_filter_ready           # 
int8 is_filter_ok              # 
int8 is_heading_determined     # false - heading is unknown yet
int8 is_stationary             # true - stationary, false - dynamic
int8 gnss_status               # 1-Standalone, 3-DGNSS, 5-RTK FLOAT, 4-RTK FIXED
int8 is_gnss_ok                # 0 - gnss fault. Do not use GNSS measurement
int8 is_odom_ok
int8 is_waypoint_away          # 1 - far frome waypoint
int8 is_waypoint_heading_diff  # 1- heading is different with waypoint heading

float64 x                      # ECEF coordinate X [m]
float64 y                      # ECEF Y [m]
float64 z                      # ECEF Z [m]

float64 local_N                # NED coordinate from origin [m]
float64 local_E                # NED coordinate from origin [m]
float64 local_D                # NED coordinate from origin [m]

float64 vel_x                  # ECEF X velocity [m/s]
float64 vel_y                  # ECEF Y velocity [m/s]
float64 vel_z                  # ECEF Z velocity [m/s]

float64 vel_N                  # NED N velocity [m/s]
float64 vel_E                  # NED E velocity [m/s]
float64 vel_D                  # NED D velocity [m/s]

float64 roll_deg               # in NED coordinates [deg]
float64 pitch_deg              # in NED coordinates [deg]
float64 heading_deg            # yaw in NED coordinates[deg, 0deg at true north]

float64 acc_bias_x              # accel. bias [m/s2]
float64 acc_bias_y              # accel. bias [m/s2]
float64 acc_bias_z              # accel. bias [m/s2]

float64 gyro_bias_x             # gyro bias [deg/s]
float64 gyro_bias_y             # gyro bias [deg/s]
float64 gyro_bias_z             # gyro bias [deg/s]
                                    
# TODO is it neccessary to publish coavriances?
#float64 cov_pos_x              # Variance of position x 
#float64 cov_pos_y              # Variance of position y
#float64 cov_pos_z              # Variance of position z
#float64 cov_vel_x              # Variance of velocity x
#float64 cov_vel_y              # Variance of velocity y 
#float64 cov_vel_z              # Variance of velocity z
#float64 cov_theta_x            # Variance of axis angle x
#float64 cov_theta_y            # Variance of axis angle y 
#float64 cov_theta_z            # Variance of axis angle z
#float64 cov_acc_bias_x         # Variance of acc bias  x 
#float64 cov_acc_bias_y         # Variance of acc bias  y
#float64 cov_acc_bias_z         # Variance of acc bias  z
#float64 cov_gyro_bias_x        # Variance of gyro bias x 
#float64 cov_gyro_bias_y        # Variance of gyro bias y
#float64 cov_gyro_bias_z        # Variance of gyro bias z

uint8 num_sat                   # Number of Satellites used in navigation solution
