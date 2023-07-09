uint64 timestamp		# time since system start (microseconds)

uint8 state			# state of the flight test input
uint8 mode			# input mode of the flight test input
uint8 excite_point		# excitation point for the flight test input
uint8 excite_index		# excitation index for the flight test input

float32 sweep_input		# The frequency sweep input without noise.
float32 doublet_input		# The doublet pulse at the current time step for the flight test input.
float32 excite_input		# The excite input. at each time step in the Sweep or Doublet input.

float32 manual_sp		# The manual input setpoint.
float32 excite_sp		# The excite input setpoint is the result of summing the manual input setpoint and the excite input.
