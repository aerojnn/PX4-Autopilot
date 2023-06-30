uint64 timestamp			# time since system start (microseconds)

float32 indicated_airspeed_m_s   	# indicated airspeed in m/s
float32 calibrated_airspeed_m_s     	# calibrated airspeed in m/s (CAS, accounts for instrumentation errors), set to NAN if invalid
float32 true_airspeed_m_s		# true filtered airspeed in m/s (TAS), set to NAN if invalid

float32 aoa_deg       			# Normalize angle of attack (alpha) in Degree (-180,180)
float32 aos_deg       			# Normalize angle of sideslip (beta) in Degree (-180,180)

float32 air_temperature_celsius  	# air temperature in degrees celsius, -1000 if unknown

float32 ref_angle_deg       		# Reference angle for wind tunnel testing
float32 ref_airspeed_m_s       		# Reference airspeed for wind tunnel testing
