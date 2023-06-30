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

#include "AS5600L0.hpp"

using namespace time_literals;

AS5600L0::AS5600L0(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_keep_retrying(config.keep_running)
{
}

AS5600L0::~AS5600L0()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int AS5600L0::init()
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

void AS5600L0::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

int AS5600L0::probe()
{
	_retries = 1;
	bool require_initialization = !init_as5600l();

	if (require_initialization && _keep_retrying) {
		PX4_INFO("no sensor found, but will keep retrying");
		return 0;
	}

	return require_initialization ? -1 : 0;
}

int AS5600L0::read(unsigned address, void *data, unsigned count)
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

int AS5600L0::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

bool AS5600L0::init_as5600l()
{
	uint8_t status;
	int ret = read(AS5600L_STATUS, &status, sizeof(status));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("readStatus : i2c::transfer returned %d", ret);
		return false;
	}

	uint8_t rawAngle[2];
	ret = read(AS5600L_RAW_ANGLE, &rawAngle, sizeof(rawAngle));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("rawAngle : i2c::transfer returned %d", ret);
		return false;
	}

	_z_position = rawAngle[0] << 8 | rawAngle[1];

	if ((status & AS5600L_MAGNET_DETECT) > 1) {
		// ret = setZPosition(_z_position);
		ret = setZPosition(_param_z_position.get());
		return ret;
	} else {
		perf_count(_comms_errors);
		DEVICE_DEBUG("no magnet was detected");
		return false;
	}
}

bool AS5600L0::setZPosition(uint16_t value)
{
	if (value > 0xfff) {
		return false;
	} else {
		uint8_t zpos[2];
		zpos[0] = value >> 8;
		zpos[1] = value & 0xff;
		int ret = write(AS5600L_START_POSITION, &zpos, sizeof(zpos));

		if (ret != PX4_OK) {
			perf_count(_comms_errors);
			PX4_DEBUG("setZPosition : i2c::transfer returned %d", ret);
		}

		return ret == PX4_OK;
	}
}

void AS5600L0::RunImpl()
{
	readAngle();

	ScheduleDelayed(CONVERSION_INTERVAL);
}

int AS5600L0::readAngle()
{
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// read 2 bytes from the sensor
	uint8_t val[2];
	int ret = read(AS5600L_ANGLE, &val, sizeof(val));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("readAngle : i2c::transfer returned %d", ret);
		return ret;
	}

	uint16_t _scale_angle = val[0] << 8 | val[1];

	float angle_degree = (float)_scale_angle * _angle_to_degrees;
	float nor_degree = fmod(angle_degree + 180, 360) - 180;

	if (_param_pol_angle.get() == 1){
		nor_degree = nor_degree * -1.0f;
	}

	sensor_position_s msg{};
	msg.timestamp_sample = timestamp_sample;
	msg.raw_angle = _z_position;
	msg.scale_angle = _scale_angle;
	msg.nor_angle = nor_degree;
	msg.error_count = perf_event_count(_comms_errors);
	msg.timestamp = hrt_absolute_time();
	_aoa_pub.publish(msg);

	perf_end(_sample_perf);

	return ret;
}

