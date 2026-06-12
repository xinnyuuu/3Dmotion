# Timestamp is represented as microseconds to match the WAM token stream.
uint64 timestamp_us
string frame_id
string child_frame_id

float64[3] position
float64[4] orientation_wxyz
float64[3] linear_velocity
float64[3] angular_velocity

uint8 tracking_state

