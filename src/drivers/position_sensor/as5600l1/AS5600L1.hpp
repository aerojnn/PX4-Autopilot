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
 * @file AS5600L1.hpp
 *
 * @author Nanthawat Saetun <Nanthawat.jn@gmail.com>
 *
 * Driver AMS AS5600L Magnetic Rotary Position Sensor for Side Slip Angle
 *
 * Datasheet: https://ams.com/documents/20143/36005/AS5600L_DS000545_3-00.pdf
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_position.h>

#define I2C_ADDRESS_DEFAULT_AS5600L 0x40
#define I2C_ADDRESS_PROGRAM_AS5600L 0x41

static constexpr uint32_t 	I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface

// Configuration Registers
#define AS5600L_START_POSITION 	0x01 // + 0x02 (LSB)

// Output Register
#define AS5600L_RAW_ANGLE 	0x0C // + 0x0D (LSB)
#define AS5600L_ANGLE		0x0E // + 0x0F (LSB)

// Status Register
#define AS5600L_STATUS		0x0B

// Status Bits
#define AS5600L_MAGNET_DETECT 	0x20

// Measurement rate is 200Hz
#define AL5600L_MEAS_RATE 	200
#define CONVERSION_INTERVAL	(1000000 / AL5600L_MEAS_RATE)	/* microseconds */

class AS5600L1:  public device::I2C, public ModuleParams, public I2CSPIDriver<AS5600L1>
{
public:
	AS5600L1(const I2CSPIDriverConfig &config);
	~AS5600L1() override;

	static void print_usage();

	int init() override;
	void print_status() override;

	void RunImpl();

private:
	int probe() override;

	bool init_as5600l();
	bool setZPosition(uint16_t value);

	int readAngle();

	int read(unsigned address, void *data, unsigned count);
	int write(unsigned address, void *data, unsigned count);

	uint16_t _z_position{};
	const float _angle_to_degrees = 360.0f / 4095;
	const bool _keep_retrying;

	uORB::PublicationMulti<sensor_position_s> _aoa_pub{ORB_ID(sensor_position)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AOS_Z_POSITION>) _param_z_position,
		(ParamInt<px4::params::POLAR_AOS_ANGLE>) _param_pol_angle
	)
};
