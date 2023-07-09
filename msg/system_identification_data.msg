uint64 timestamp	# time since system start (microseconds)

float32 airspeed	# True airspeed
float32 alpha		# Air boom data: angle of attack (AOA)
float32 beta		# Air boom data: angle of sideslipe (AOS)

float32 phi		# Attitudes (Euler angles): roll (+ right wing down)
float32 theta		# Attitudes (Euler angles): pitch (+ nose up)
float32 psi		# Attitudes (Euler angles): yaw (+ nose right)

float32 p		# Angular rates around body axis x: roll rate (+ right wing down)
float32 q		# Angular rates around body axis y: pitch rate (+ nose up)
float32 r		# Angular rates around body axis z: yaw rate (+ nose right)

float32 ax		# Acceleration along the body axis x: longitudinal
float32 ay		# Acceleration along the body axis y: lateral
float32 az		# Acceleration along the body axis z: vertical

float32 d_a		# Control inputs for fixed-wing aircraft: roll (aileron deflection)
float32 d_e		# Control inputs for fixed-wing aircraft: pitch (elevator deflection)
float32 d_r		# Control inputs for fixed-wing aircraft: yaw (rudder deflection)
float32 d_t		# Control inputs for fixed-wing aircraft: throttle controls

# Additional
float32 d_rpm		# Control input for motor rpm

