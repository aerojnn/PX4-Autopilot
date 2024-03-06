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

#include "AirdataBoom.hpp"

AirdataBoom::AirdataBoom() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

AirdataBoom::~AirdataBoom()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool AirdataBoom::init()
{
	// alternatively, Run on fixed interval
	// ScheduleOnInterval(SCHEDULE_INTERVAL);

	return true;
}

void AirdataBoom::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	//  poll airdata of the sensor
	if (_sensor_position_sub[0].update(&aoa) && _sensor_position_sub[1].update(&aos)
	    	&& _airspeed_sub[0].update(&airspeed_raw)
	    	&& _airspeed_validated_sub.update(&airspeed_validated))
	{
		// publish data
		publish();
	}

	perf_end(_loop_perf);
}

void AirdataBoom::publish()
{
	airdata_boom_s airdata_boom{};
	airdata_boom.timestamp = hrt_absolute_time();
	airdata_boom.indicated_airspeed_m_s = airspeed_raw.indicated_airspeed_m_s;
	airdata_boom.calibrated_airspeed_m_s = airspeed_validated.calibrated_airspeed_m_s;
	airdata_boom.true_airspeed_m_s = airspeed_validated.true_airspeed_m_s;
	airdata_boom.aoa_deg = aoa.nor_angle;
	airdata_boom.aos_deg = aos.nor_angle;
	airdata_boom.air_temperature_celsius = airspeed_raw.air_temperature_celsius;

	_airdata_boom_pub.publish(airdata_boom);
}

int AirdataBoom::task_spawn(int argc, char *argv[])
{
	AirdataBoom *instance = new AirdataBoom();

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

int AirdataBoom::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int AirdataBoom::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AirdataBoom::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provides a single airdata_boom topic, containing angle of attack (AOA),
sideslip angle (SSA), true airspeed (TAS) and air temperature in .

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("airdata_boom", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int airdata_boom_main(int argc, char *argv[])
{
	return AirdataBoom::main(argc, argv);
}
