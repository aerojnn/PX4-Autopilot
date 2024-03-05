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
 * @file SystemIdentificationData.cpp
 *
 * @author Nanthawat Saetun <nanthawat.jn@gmail.com>
 */

#include "SystemIdentificationData.hpp"

SystemIdentificationData::SystemIdentificationData() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

SystemIdentificationData::~SystemIdentificationData()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool SystemIdentificationData::init()
{
	int rate = _param_sys_iden_interval.get();

	// default to 250 Hz (4000 us interval)
	if (rate <= 0) {
		rate = 20;
	}

	// 10 - 500 Hz
	int interval_us = constrain(int(roundf(1e6f / rate)), 2000, 100000);

	// alternatively, Run on fixed interval
	ScheduleOnInterval(interval_us);

	return true;
}

void SystemIdentificationData::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	_instrument_sub.update(&servo_deg);
	_actuator_controls_sub.update(&control_input);

	// Measure servo deflection (output) in deg from Arduino
	_def_deg	= servo_deg.angle;

	// Aerodynamic control surface deflection [-1, 1]
	_def_roll	= control_input.control[0];
	_def_pitch	= control_input.control[1];
	_def_yaw	= control_input.control[2];

	// publish data
	publish();


	perf_end(_loop_perf);
}

void SystemIdentificationData::publish()
{
	system_identification_data_s 	sys_iden_data{};

	sys_iden_data.timestamp = hrt_absolute_time();
	sys_iden_data.servo	= _def_deg;
	sys_iden_data.d_a	= _def_roll;
	sys_iden_data.d_e	= _def_pitch;
	sys_iden_data.d_r	= _def_yaw;

	_system_identification_data_pub.publish(sys_iden_data);
}

int SystemIdentificationData::task_spawn(int argc, char *argv[])
{
	SystemIdentificationData *instance = new SystemIdentificationData();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SystemIdentificationData::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int SystemIdentificationData::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SystemIdentificationData::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provides a single system_identification_data topic, containing from flight tests
for system identification purposes.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("system_identification_data", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int system_identification_data_main(int argc, char *argv[])
{
	return SystemIdentificationData::main(argc, argv);
}
