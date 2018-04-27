Header header
float64 pos_x # meters
float64 pos_y # meters
float64 pos_x_self
float64 pos_y_self
float64 pos_z_self
float64 pos_theta_self
float64 pos_theta # rads
float64[6] distance # meters
uint8 num
# num represents the number of base stations, range from 0 to 6
bool check_failed
# check_failed = true if self check failed
bool need_calibrated
# need_calibrated = true if magnetic sensor needs calibrated
uint8 level
# level represents how strong the signal is, range from 0 to 3, the larger the better
