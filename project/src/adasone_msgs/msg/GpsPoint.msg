Header header

uint32 iTOW         # GPS Millisecond time of week [ms]
int8 status         # 1-Standalone, 3-DGNSS, 5-RTK FLOAT, 4-RTK FIXED

float64 lat         # Longitude [deg]
float64 lon         # Latitude [deg]
float64 height      # Height [m]
float64 hMSL        # hMSL [m]

float64 x           # ECEF coordinate X [m]
float64 y           # ECEF Y [m]
float64 z           # ECEF Z [m]

float64 local_N     # NED coordinate x from certain origin [m]
float64 local_E     # NED coordinate y from certain origin [m]
float64 local_D     # NED coordinate z from certain origin [m]

float64 vel_x       # ECEF X velocity [m/s]
float64 vel_y       # ECEF Y velocity [m/s]
float64 vel_z       # ECEF Z velocity [m/s]

float64 vel_N       # NED east velocity [m/s]
float64 vel_E       # NED north velocity [m/s]
float64 vel_D       # NED up velocity [m/s]

float64 headingDeg  # Heading angle from ublox [0~360deg, 0deg at true north]
float64 headingAcc  # Heading angle accuracy

float64 covH        # Variance of horizontal position from horizontal accuracy estimate
float64 covV        # Variance of vertial position
float64 covSpeed    # Variance of Speed
float64 covHeading  # Variance of Heading

uint8 numsat        # Number of Satellites used in navigation solution

bool speed_bump     # Speed Bump detection
bool obu_rtcm_is_alive
bool obu_ccan_is_alive
bool obu_ethernet_is_alive