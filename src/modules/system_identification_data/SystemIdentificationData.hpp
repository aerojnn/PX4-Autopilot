/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file SystemIdentificationData.hpp
 *
 * This module provides collects data from flight tests for system identification purposes.
 *
 * @author Nanthawat Saetun <nanthawat.jn@gmail.com>
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/system_identification_data.h>
#include <uORB/topics/flight_test_input.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/airdata_boom.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/rpm.h>

using matrix::Eulerf;
using matrix::Quatf;
using math::constrain;
using math::degrees;

using namespace time_literals;

class SystemIdentificationData : public ModuleBase<SystemIdentificationData>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	SystemIdentificationData();
	~SystemIdentificationData() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _flight_test_input_sub{ORB_ID(flight_test_input)};			/**< flight test input */

	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};			/**< true airspeed */
	uORB::Subscription _airdata_boom_pub{ORB_ID(airdata_boom)};				/**< alpha and beta */
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};			/**< attitude (euler) angles*/
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};	/**< angular rates */
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};		/**< angular rates */
	uORB::Subscription _actuator_controls_sub{ORB_ID(actuator_controls_0)};			/**< control inputs */
	uORB::Subscription _rpm_sub{ORB_ID(rpm)};						/**< motor rpm inputs */

	// Publications
	uORB::Publication<system_identification_data_s> _system_identification_data_pub{ORB_ID(system_identification_data)};

	flight_test_input_s		fti;
	airspeed_validated_s 		airspeed;
	airdata_boom_s			airboom_data;
	vehicle_attitude_s		attitude;
	vehicle_angular_velocity_s	angular_rate;
	vehicle_acceleration_s		acceleration;
	actuator_controls_s		control_input;
	rpm_s				rpm;

	float _airspeed{0};
	float _aoa{0};
	float _aos{0};
	float _roll_deg{0};
	float _pitch_deg{0};
	float _yaw_deg{0};
	float _roll_rate_deg{0};
	float _pitch_rate_deg{0};
	float _yaw_rate_deg{0};
	float _ax{0};
	float _ay{0};
	float _az{0};
	float _def_roll{0};
	float _def_pitch{0};
	float _def_yaw{0};
	float _def_throttle{0};
	float _def_rpm{0};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYSID_RATE>) 	_param_interval,   /**< interval parameter */
		(ParamFloat<px4::params::SYSID_MAX_AIL_D>)    	_param_max_ail_def,
		(ParamFloat<px4::params::SYSID_MAX_ELE_D>)    	_param_max_ele_def,
		(ParamFloat<px4::params::SYSID_MAX_RUD_D>)    	_param_max_rud_def
	)

	void publish(); /**< publish data */
};
