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
 * @file ATMEGA2560.hpp
 *
 * @author Nanthawat Saetun <Nanthawat.jn@gmail.com>
 *
 * Driver for Reference Instrument using Atmega2560 collects data.
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/instrument.h>

#define I2C_ADDRESS 0x51

static constexpr uint32_t 	I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface

// Output Register
#define REG_DATA 		0x01

// Measurement rate is 100Hz
#define ATMEGA2560_MEAS_RATE 	100
#define CONVERSION_INTERVAL	(1000000 / ATMEGA2560_MEAS_RATE)	/* microseconds */

class ATMEGA2560:  public device::I2C, public ModuleParams, public I2CSPIDriver<ATMEGA2560>
{
public:
	ATMEGA2560(const I2CSPIDriverConfig &config);
	~ATMEGA2560() override;

	static void print_usage();

	int init() override;
	void print_status() override;

	void RunImpl();

private:
	int probe() override;

	bool init_atmega2560();

	int collect();

	int read(unsigned address, void *data, unsigned count);

	const bool _keep_retrying;

	uORB::Publication<instrument_s> _instrument_pub{ORB_ID(instrument)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::POLAR_REF_ANGLE>) _param_pol_ref_angle
	)
};
