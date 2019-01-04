# Qualcom Machine Vision VISLAM status
uint64 timestamp		# time since system start (microseconds)

int32 status
int32 quality
uint32 error_code
uint32 tracked_points # tracked point count

float32 time_alignment
float32[3] gravity
float32[3] w_bias
float32[3] a_bias
float32[9] rbg # Estimated accelorometer gyro rotation matrix 
float32[3] tbc
# estimated camera IMU transformation matrix
float32[9] rbc
# camera IMU transformation in axis angle form
float32[3] rbc_aa
