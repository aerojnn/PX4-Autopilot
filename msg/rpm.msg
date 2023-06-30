uint64 timestamp                      # time since system start (microseconds)
uint64 timestamp_sample

float32 indicated_frequency_rpm       # indicated rotor Frequency in Revolution per minute
float32 estimated_accurancy_rpm       # estimated accurancy in Revolution per minute

float32[5] electrical_speed_rpm		# multi motor electrical Speed in Revolution per minute, maximum of 5 motors

uint32 error_count			# Number of errors detected by driver
