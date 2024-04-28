uint64 timestamp	# time since system start (microseconds)

float32 airspeed	# [m/s] True airspeed
float32 alpha		# [deg] Air boom data: angle of attack (AOA)
float32 beta		# [deg] Air boom data: angle of sideslipe (AOS)

float32 phi		# [deg] Attitudes (Euler angles): roll (+ right wing down)
float32 theta		# [deg] Attitudes (Euler angles): pitch (+ nose up)
float32 psi		# [deg] Attitudes (Euler angles): yaw (+ nose right)

float32 p		# [deg/s] Angular rates around body axis x: roll rate (+ right wing down)
float32 q		# [deg/s] Angular rates around body axis y: pitch rate (+ nose up)
float32 r		# [deg/s] Angular rates around body axis z: yaw rate (+ nose right)

float32 ax		# [m/s^2] Acceleration along the body axis x: longitudinal
float32 ay		# [m/s^2] Acceleration along the body axis y: lateral
float32 az		# [m/s^2] Acceleration along the body axis z: vertical

float32 d_a		# [deg] Control inputs for fixed-wing aircraft: roll (aileron deflection)
float32 d_e		# [deg] Control inputs for fixed-wing aircraft: pitch (elevator deflection)
float32 d_r		# [deg] Control inputs for fixed-wing aircraft: yaw (rudder deflection)
float32 d_t		# [deg] Control inputs for fixed-wing aircraft: throttle controls

# Additional
float32 d_rpm		# Control input for motor rpm

