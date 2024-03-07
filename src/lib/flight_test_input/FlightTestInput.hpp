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
 * @file FlightTestInput.hpp
 *
 * This library provides control setpoint from excitation input during flight tests
 * for system identification purposes.
 *
 * @author Nanthawat Saetun <nanthawat.jn@gmail.com>
 *
 * Acknowledgements:
 *   This library is modified based on the library by Prioria Robotics and Watcharapol Saengphet.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/flight_test_input.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/vehicle_status.h>

#include <mathlib/mathlib.h>

using namespace time_literals;

class FlightTestInput : public ModuleParams
{
public:
	FlightTestInput();
	~FlightTestInput() override = default;

	/**
	 * Update test input computation
	*/
	void 	fti_update(const float dt, const uint8_t excite_point);

	/**
	 * Excitation input
	 */
	float 	excite(const uint8_t excite_index, const float manual_sp);

private:
	/**
	 * Subscriptions
	*/
	uORB::SubscriptionInterval 	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription 		_vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};		/**< Vehicle status subscription */
	uORB::Subscription 		_vehicle_status_sub{ORB_ID(vehicle_status)};			/**< Vehicle status subscription */
	uORB::Subscription 		_commander_state_sub{ORB_ID(commander_state)};			/**< Commander state subscription */

	/**
	 * Publications
	*/
	uORB::Publication<flight_test_input_s>	_flight_test_input_pub{ORB_ID(flight_test_input)};
	orb_advert_t			_mavlink_log_pub{nullptr};					/**< Mavlink log uORB handle */

	vehicle_control_mode_s	_vcontrol_mode;		/**< vehicle control mode */
	vehicle_status_s	_vehicle_status;	/**< vehicle status */
	commander_state_s 	_commander_state;	/**< commander state */

	enum FlightTestInputState {
		TEST_INPUT_OFF = 0,
		TEST_INPUT_INIT,
		TEST_INPUT_RUNNING,
		TEST_INPUT_COMPLETE
	} _state;

	const float c1 = 4.0f;
	const float c2 = 0.0187f;
	float k = 0.0f;

	float _time_running;		/**< Time */

	float omega_min{0};		/**< Minimum frequency sweep */
	float omega_max{0};		/**< Maximmum frequency sweep */
	float sweep_amp{0};		/**< Frequency sweep amplitude */
	float t_rec{0}; 		/**< Duration for frequency progression. */
	float t_max{0}; 		/**< Duration for long-period input. */
	float t_trim{0}; 		/**< Duration for trim condition. */
	float t_fade_in{0}; 		/**< Duration for fade-in at the initial starting frequency. */
	float t_fade_out{0}; 		/**< Duration for fade-out at the final ending frequency. */

	float t_test{0}; 		/**< Duration for excitation inputs testing */

	float _sweep_freq{0};		/**< Frequency sweep */
	float _sweep_amp{0};		/**< Excitation sweep inputs amplitude */
	float _sweep_input{0};		/**< Sweep inputs */

	float _doublet_amp{0};		/**< Doublet pulse amplitude */
	float _doublet_length{0};	/**< Doublet pulse amplitude */
	float _doublet_input{0};		/**< Doublet pulse inputs */

	float _excite_input{0};		/**< Excitation inputs */
	float _manual_sp{0};
	float _excite_sp{0};

	/**
	 * System main_state and nav_state captured during test input init
	 */
	uint8_t _main_state;
	uint8_t _nav_state;

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void 	parameters_update();

	/**
	 * Test input computation
	*/
	void 	compute_sweep(float dt);
	void 	compute_doublet(float dt);

	/**
	 * Publish flight test input data
	*/
	void	publish_data();

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FTI_ENABLE>) 		_param_enable,			/**< Main parameters */
		(ParamInt<px4::params::FTI_MODE>) 		_param_mode,			/**< Main parameters */
		(ParamInt<px4::params::FTI_EXCITE_POINT>) 	_param_excite_point,		/**< Main parameters */
		(ParamInt<px4::params::FTI_EXCITE_INDEX>) 	_param_excite_index,		/**< Main parameters */
		(ParamFloat<px4::params::FTI_FS_FREQ_MIN>) 	_param_freq_min,   		/**< Frequency sweep parameters */
		(ParamFloat<px4::params::FTI_FS_FREQ_MAX>) 	_param_freq_max,   		/**< Frequency sweep parameters */
		(ParamFloat<px4::params::FTI_FS_SWEEP_AMP>) 	_param_sweep_amp,   		/**< Frequency sweep parameters */
		(ParamFloat<px4::params::FTI_FS_T_TRIM>) 	_param_t_trim,   		/**< Frequency sweep parameters */
		(ParamFloat<px4::params::FTI_FS_T_FADE_I>) 	_param_t_fade_in,   		/**< Frequency sweep parameters */
		(ParamFloat<px4::params::FTI_FS_T_FADE_O>) 	_param_t_fade_out,   		/**< Frequency sweep parameters */
		(ParamFloat<px4::params::FTI_PULSE_LEN>) 	_param_doublet_length,   	/**< Doublet parameters */
		(ParamFloat<px4::params::FTI_PULSE_AMP>) 	_param_doublet_amp		/**< Doublet parameters */
	)
};
