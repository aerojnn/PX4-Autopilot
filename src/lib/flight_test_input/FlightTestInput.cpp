/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file FlightTestInput.cpp
 *
 */

#include "FlightTestInput.hpp"

FlightTestInput::FlightTestInput() :
	ModuleParams(nullptr),
	_state(TEST_INPUT_OFF)
{
	_param_enable.set(false);
	_param_enable.commit();
}

void
FlightTestInput::parameters_update()
{
	/**
	 *  Check if parameters have changed
	*/
	if (_parameter_update_sub.updated())
	{
		/**
		 * Clear update
		*/
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		/**
		 * Update module parameters (in DEFINE_PARAMETERS).
		 *
		 * If any parameter updated, call updateParams() to check if
		 * this class attributes need updating (and do so).
		*/
		updateParams();
	}
}

void
FlightTestInput::fti_update(const float dt, const uint8_t excite_point)
{
	parameters_update();

	switch (_state)
	{
	case TEST_INPUT_OFF:

		if (_param_enable.get())
		{
			_state = TEST_INPUT_INIT;

			/**
			 * Check vehicle status for changes to publication state
			 */
			_vehicle_status_sub.update(&_vehicle_status);
			_commander_state_sub.update(&_commander_state);
			_vcontrol_mode_sub.update(&_vcontrol_mode);
		}

		break;

	case TEST_INPUT_INIT:
		/**
		 * Initialize sweep variables and store current autopilot mode
		 */

		mavlink_log_info(&_mavlink_log_pub, "#Flight test input excitation starting");

		/**
		 * Abort sweep if any system mode (main_state or nav_state) change
		*/
		_main_state = _commander_state.main_state;
		_nav_state = _vehicle_status.nav_state;

		_state = TEST_INPUT_RUNNING;

		_time_running = 0;
		_sweep_input = 0;
		_doublet_input = 0;
		_excite_input = 0;

		break;

	case TEST_INPUT_RUNNING:

		/**
		 * Only run if main state and nav state are unchanged and sweep mode is valid
		*/

		if ((_main_state == _commander_state.main_state)
		    && (_nav_state == _vehicle_status.nav_state)
		    && (_param_enable.get())
		    && (_param_mode.get() == 0 || _param_mode.get() == 1)
		    && (excite_point == _param_excite_point.get()))
		    {
			if (_param_mode.get() == 0)
			{
				compute_sweep(dt);	/**< Frequency sweep mode */
			} else if (_param_mode.get() == 1)
			{
				compute_doublet(dt);	/**< Doublet mode */
			}

			_time_running += dt;		/**< Increment time */

		} else
		{
			mavlink_log_info(&_mavlink_log_pub, "#Flight test input aborted");
			_state = TEST_INPUT_COMPLETE;
		}

		break;

	case TEST_INPUT_COMPLETE:

		_param_enable.set(false);
		_param_enable.commit();

		/**
		 * Only return to off state once param is reset to 0
		*/
		if (!_param_enable.get()) {
			mavlink_log_info(&_mavlink_log_pub, "#Flight test input resetting");
			_state = TEST_INPUT_OFF;
		}

		break;
	}

	publish_data();
}

void
FlightTestInput::compute_sweep(float dt)
{
	/**
	 * Design Frequency Sweep
	 *
	 * @param _freq_hz	[Hz] Frequency sweep
	 *
	 * @param _t_test	[hr] Maximum time for long-period frequency sweep inputs.
	 *
	 * @param _sweep_amp 	[pct] Frequency sweep amplitude, typically 10% of the maximum deflection limits.
	 * 			The manual control setpoint range, which is from -1 to 1.
	*/

	/**
	 * Computer-Generated Sweeps
	 *
	 * Consists of:
	 * 	- Trim duration before starting frequency sweep
	 * 	- Constant frequency of omega_min (One full long-period inputs)
	 * 	- Trim duration after ending
	*/

	_freq_hz = _param_freq_hz.get();
	_t_test = _param_t_test.get();

	/** frequency sweep input parameter */
	_t_trim_start 	= _param_t_trim_start.get();
	_t_trim_end 	= _param_t_trim_end.get();
	_sweep_amp	= _param_sweep_amp.get();	/**< Typically 10% of the max. deflection limits, which is from -1 to 1. */

	/** Excite input duration for frequency sweep input. */
	_t_total 		= _t_trim_start + _t_test + _t_trim_end;
	float t_run2t_trim_in 	= _t_trim_start;
	float t_run2t_test 	= _t_trim_start + _t_test;

	if (_time_running <= _t_total)
	{
		if (_time_running <= t_run2t_trim_in) 	/**< Trim duration before the starting sweep */
		{
			_excite_sweep_amp = 0.0f;
			_freq_sweep = 0.0f;

		} else if ((_time_running > t_run2t_trim_in) && (_time_running <= t_run2t_test)) /**< Constant frequency duration for one long-period */
		{
			_excite_sweep_amp = _sweep_amp;

			float omega = _freq_hz * float(2 * M_PI);
			_freq_sweep += omega * dt;
		} else 	/**< Trim duration after the ending sweep */
		{
			_excite_sweep_amp = 0.0f;
			_freq_sweep = 0.0f;
		}

		_sweep_input = _excite_sweep_amp * sinf(_freq_sweep); 	/**< Frequency sweep inputs */
		_excite_input = _sweep_input;				/**< Excitation inputs */

	} else
	{
		mavlink_log_info(&_mavlink_log_pub, "#Frequency Sweep complete");
		_state = TEST_INPUT_COMPLETE;
	}
}

void
FlightTestInput::compute_doublet(float dt)
{
	/**
	 * Two-sided Doublet Inputs
	 *
	 *           FTI_PULSE_LEN
	 *           |~~~~~~~~~|
	 *           |         |
	 *           |         |
	 *     1s    |         |             1s
	 *  ~~~~~~~~~|         |         |~~~~~~~~~~
	 *                     |         |
	 *                     |         |
	 *                     |         |
	 *                     |~~~~~~~~~|
	 *                     FTI_PULSE_LEN
	 *
	*/

	/** doublet input parameter */
	_doublet_length	= _param_doublet_length.get();
	_doublet_amp 	= math::constrain(_param_doublet_amp.get(), -1.0f, 1.0f);

	const float lead_in_time = 1.0f;
	const float lead_out_time = 1.0f;

	_t_total 		= lead_in_time + (2.0f * _doublet_length) + lead_out_time;
	float t_run2t_one_side	= lead_in_time + _doublet_length;
	float t_run2t_two_side 	= lead_in_time + (2.0f * _doublet_length);

	if (_time_running <= _t_total)
	{
		if (_time_running <= lead_in_time)	/**< Lead-in */
		{
			_doublet_input = 0.0f;

		} else if ((_time_running > lead_in_time) && (_time_running <= t_run2t_one_side)) 	/**< One-sided Doublet */
		{
			_doublet_input = _doublet_amp;

		} else if ((_time_running > t_run2t_one_side) && (_time_running <= t_run2t_two_side)) 	/**< One-sided Doublet */
		{
			_doublet_input = -(_doublet_amp);

		} else 	/**< Lead-out*/
		{
			_doublet_input = 0.0f;
		}

		_excite_input = _doublet_input;		/**< Excitation inputs */

	} else
	{
		mavlink_log_info(&_mavlink_log_pub, "#Doublet complete");
		_state = TEST_INPUT_COMPLETE;
	}
}

float
FlightTestInput::excite(const uint8_t excite_index, const float manual_sp)
{
	_excite_sp = manual_sp;

	if ((_state == TEST_INPUT_RUNNING) && (excite_index == _param_excite_index.get()))
	{
		_manual_sp = manual_sp;
		_excite_sp = manual_sp + _excite_input;
	}

	return _excite_sp;
}

void
FlightTestInput::publish_data()
{
	flight_test_input_s 	_flight_test_input{};	/**< flight test input */

	_flight_test_input.timestamp = hrt_absolute_time();
	_flight_test_input.state = _state;
	_flight_test_input.mode = _param_mode.get();
	_flight_test_input.excite_point = _param_excite_point.get();
	_flight_test_input.excite_index = _param_excite_index.get();
	_flight_test_input.sweep_input = _sweep_input;
	_flight_test_input.doublet_input = _doublet_input;
	_flight_test_input.excite_input = _excite_input;
	_flight_test_input.manual_sp = _manual_sp;
	_flight_test_input.excite_sp = _excite_sp;

	_flight_test_input_pub.publish(_flight_test_input);
}
