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

#include "ATMEGA4809.hpp"

using namespace time_literals;

ATMEGA4809::ATMEGA4809(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_keep_retrying(config.keep_running)
{
}

ATMEGA4809::~ATMEGA4809()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int ATMEGA4809::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	return ret;
}

void ATMEGA4809::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

int ATMEGA4809::probe()
{
	_retries = 1;
	bool require_initialization = !init_atmega4809();

	if (require_initialization && _keep_retrying) {
		PX4_INFO("no sensor found, but will keep retrying");
		return 0;
	}

	return require_initialization ? -1 : 0;
}

int ATMEGA4809::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	int ret;

	/* We need a first transfer where we write the register to read */
	ret = transfer(&cmd, 1, nullptr, 0);

	if (ret != OK) {
		return ret;
	}

	/* Now we read the previously selected register */
	ret = transfer(nullptr, 0, (uint8_t *)data, count);

	return ret;
}

bool ATMEGA4809::init_atmega4809()
{
	uint8_t cmd[1];
	cmd[0] = 0x00;

	// Send the command to begin a measurement.
	int ret = transfer(cmd, 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return ret == PX4_OK;
}

void ATMEGA4809::RunImpl()
{
	collect();

	ScheduleDelayed(CONVERSION_INTERVAL);
}

int ATMEGA4809::collect()
{
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// read 10 bytes from the sensor
	uint8_t val[10];
	int ret = read(REG_FREQEUNCY, &val, sizeof(val));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("readAngle : i2c::transfer returned %d", ret);
		return ret;
	}

	uint16_t _freq[5];
	_freq[0] = val[0] << 8 | val[1];	// motor1
	_freq[1] = val[2] << 8 | val[3];	// motor2
	_freq[2] = val[4] << 8 | val[5];	// motor3
	_freq[3] = val[6] << 8 | val[7];	// motor4
	_freq[4] = val[8] << 8 | val[9];	// motor5 (main)

	float speed_rpm[5];
	for (int i = 0; i < 5; i++) {
		if (i == 5) {
			speed_rpm[i] = 120 * _freq[i] / _param_main_poles.get();
		} else {
			speed_rpm[i] = 120 * _freq[i] / _param_poles.get();
		}
	}

	rpm_s msg{};

	for (int i = 0; i < 5; i++){
		msg.electrical_speed_rpm[i] = speed_rpm[i];
	}

	msg.timestamp_sample = timestamp_sample;
	msg.error_count = perf_event_count(_comms_errors);
	msg.timestamp = hrt_absolute_time();
	_rpm_pub.publish(msg);

	perf_end(_sample_perf);

	return ret;
}

