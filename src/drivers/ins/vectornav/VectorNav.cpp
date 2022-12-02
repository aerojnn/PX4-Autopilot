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

#include "VectorNav.hpp"

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/getopt.h>

#include <fcntl.h>

VectorNav::VectorNav(const char *port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_VN300;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	_px4_accel.set_device_id(device_id.devid);
	_px4_gyro.set_device_id(device_id.devid);
	_px4_mag.set_device_id(device_id.devid);

	// uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	// if (bus_num < 10) {
	// 	device_id.devid_s.bus = bus_num;
	// }

	// _px4_rangefinder.set_device_id(device_id.devid);
	// _px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
}

VectorNav::~VectorNav()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

void VectorNav::asciiOrBinaryAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex)
{
	const hrt_abstime time_now_us = hrt_absolute_time();

	/* First make sure we have a binary packet type we expect since there
	 * are many types of binary output types that can be configured. */
	// COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL
	if ((VnUartPacket_type(packet) == PACKETTYPE_BINARY) &&
	    VnUartPacket_isCompatible(packet,
				      COMMONGROUP_NONE,
				      TIMEGROUP_NONE,
				      (ImuGroup)(IMUGROUP_ACCEL | IMUGROUP_ANGULARRATE),
				      GPSGROUP_NONE,
				      ATTITUDEGROUP_NONE,
				      INSGROUP_NONE,
				      GPSGROUP_NONE)
	   ) {

		// vec3f ypr = VnUartPacket_extractVec3f(packet);

		// char strConversions[50];
		// str_vec3f(strConversions, ypr);
		// PX4_INFO("Binary Async YPR: %s\n", strConversions);

		if (userData) {
			VectorNav *vn = static_cast<VectorNav *>(userData);

			BinaryGroupType groups = (BinaryGroupType)VnUartPacket_groups(packet);
			size_t curGroupFieldIndex = 0;

			if ((groups & BINARYGROUPTYPE_IMU) != 0) {
				ImuGroup imuGroup = (ImuGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++);

				if (imuGroup & IMUGROUP_TEMP) {
					//float temperature = VnUartPacket_extractFloat(packet);
				}

				if (imuGroup & IMUGROUP_PRES) {
					float pressure = VnUartPacket_extractFloat(packet);
					vn->PublishBaro(time_now_us, pressure);
				}

				if (imuGroup & IMUGROUP_MAG) {
					vec3f magnetic = VnUartPacket_extractVec3f(packet);
					vn->PublishMag(time_now_us, magnetic.c[0], magnetic.c[1], magnetic.c[2]);
				}

				if (imuGroup & IMUGROUP_ACCEL) {
					vec3f acceleration = VnUartPacket_extractVec3f(packet);
					vn->PublishAccel(time_now_us, acceleration.c[0], acceleration.c[1], acceleration.c[2]);
				}

				if (imuGroup & IMUGROUP_ANGULARRATE) {
					vec3f angularRate = VnUartPacket_extractVec3f(packet);
					vn->PublishGyro(time_now_us, angularRate.c[0], angularRate.c[1], angularRate.c[2]);
				}
			}
		}

		// compositeData->quaternion = VnUartPacket_extractVec4f(packet);
		// compositeData->velocityEstimatedNed = VnUartPacket_extractVec3f(packet);


		// VnCompositeData_processBinaryPacketGps2Group




	}
}

void VectorNav::PublishAccel(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	_px4_accel.update(timestamp_sample, x, y, z);
}

void VectorNav::PublishGyro(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	_px4_gyro.update(timestamp_sample, x, y, z);
}

void VectorNav::PublishMag(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	_px4_mag.update(timestamp_sample, x, y, z);
}

void VectorNav::PublishBaro(const hrt_abstime &timestamp_sample, float pressure)
{
	sensor_baro_s sensor_baro{};
	sensor_baro.device_id = 0; // TODO: DRV_INS_DEVTYPE_VN300;
	sensor_baro.pressure = pressure;
	sensor_baro.timestamp = hrt_absolute_time();

	_sensor_baro_pub.publish(sensor_baro);
}

int VectorNav::init()
{
	PX4_INFO("VectorNav::init");
	/* This example walks through using the VectorNav C Library to connect to
	 * and interact with a VectorNav sensor using the VnSensor structure. */

	/* First determine which COM port your sensor is attached to and update the
	 * constant below. Also, if you have changed your sensor from the factory
	 * default baudrate of 115200, you will need to update the baudrate
	 * constant below as well. */
	const char SENSOR_PORT[] = "/dev/ttyUSB0"; /* Linux format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
	const uint32_t SENSOR_BAUDRATE = 115200;


	VnSensor_initialize(&_vs);

	VnError error;

	/* Now connect to our sensor. */
	if ((error = VnSensor_connect(&_vs, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE) {
		PX4_ERR("Error connecting to sensor %d", error);
		return PX4_ERROR;
	}

	/* Let's query the sensor's model number. */
	char modelNumber[30];

	if ((error = VnSensor_readModelNumber(&_vs, modelNumber, sizeof(modelNumber))) != E_NONE) {
		PX4_ERR("Error reading model number %d", error);
		return PX4_ERROR;
	}

	PX4_INFO("Model Number: %s", modelNumber);




	// TODO:




	/* For the registers that have more complex configuration options, it is
	 * convenient to read the current existing register configuration, change
	 * only the values of interest, and then write the configuration to the
	 * register. This allows preserving the current settings for the register's
	 * other fields. Below, we change the heading mode used by the sensor. */
	VpeBasicControlRegister vpeReg;

	if ((error = VnSensor_readVpeBasicControl(&_vs, &vpeReg)) != E_NONE) {
		PX4_ERR("Error reading VPE basic control %d", error);
	}

	char strConversions[30] {};
	strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
	printf("Old Heading Mode: %s\n", strConversions);
	vpeReg.headingMode = VNHEADINGMODE_ABSOLUTE;

	if ((error = VnSensor_writeVpeBasicControl(&_vs, vpeReg, true)) != E_NONE) {
		PX4_WARN("Error writing VPE basic control %d", error);
	}

	if ((error = VnSensor_readVpeBasicControl(&_vs, &vpeReg)) != E_NONE) {
		PX4_WARN("Error reading VPE basic control %d", error);
	}

	strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
	printf("New Heading Mode: %s\n", strConversions);

	ImuGroup imu_group = (ImuGroup)((int)IMUGROUP_ACCEL | (int)IMUGROUP_ANGULARRATE);
	AttitudeGroup attitude_group = (AttitudeGroup)((int)ATTITUDEGROUP_VPESTATUS | (int)ATTITUDEGROUP_YAWPITCHROLL);
	InsGroup ins_group = (InsGroup)((int)INSGROUP_INSSTATUS | (int)INSGROUP_POSLLA | (int)INSGROUP_VELNED);
	GpsGroup gps_group = (GpsGroup)(GPSGROUP_UTC | GPSGROUP_TOW | GPSGROUP_NUMSATS | GPSGROUP_FIX | GPSGROUP_POSLLA |
					GPSGROUP_VELNED | GPSGROUP_TIMEU);


	// baro, mag, move later

	// 400 Hz
	BinaryOutputRegister_initialize(
		&_binary_output_400hz,
		ASYNCMODE_BOTH,
		8, // divider
		COMMONGROUP_NONE,
		TIMEGROUP_NONE,
		imu_group,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE, // attitude_group,
		INSGROUP_NONE, //ins_group,
		GPSGROUP_NONE);

	// 50 Hz (baro, mag)
	//  AttitudeGroup & InsGroup
	// InsStatus INSGROUP_INSSTATUS is a bit field ***
	BinaryOutputRegister_initialize(
		&_binary_output_50hz,
		ASYNCMODE_BOTH,
		8, // divider
		COMMONGROUP_NONE,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		attitude_group,
		ins_group,
		GPSGROUP_NONE);

	// 5 Hz GPS
	// diviser 5 hz (diviser 80)
	BinaryOutputRegister_initialize(
		&_binary_output_5hz,
		ASYNCMODE_BOTH,
		80, // divider
		COMMONGROUP_NONE,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		gps_group,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
		GPSGROUP_NONE);


	if ((error = VnSensor_writeBinaryOutput1(&_vs, &_binary_output_400hz, true)) != E_NONE) {

		// char buffer[128]{};
		// strFromVnError((char*)buffer, error);
		// PX4_ERR("Error writing binary output 1: %s", buffer);

		PX4_ERR("Error writing binary output 1 %d", error);

		return PX4_ERROR;
	}

	if ((error = VnSensor_writeBinaryOutput2(&_vs, &_binary_output_50hz, true)) != E_NONE) {
		PX4_ERR("Error writing binary output 2 %d", error);
		return PX4_ERROR;
	}

	if ((error = VnSensor_writeBinaryOutput3(&_vs, &_binary_output_5hz, true)) != E_NONE) {
		PX4_ERR("Error writing binary output 3 %d", error);
		//return PX4_ERROR;
	}

	VnSensor_registerAsyncPacketReceivedHandler(&_vs, VectorNav::asciiOrBinaryAsyncMessageReceived, this);

	return PX4_OK;
}

void VectorNav::Run()
{

	// TODO: shutdown



	// TODO: cleanup and shutdown


	// fds initialized?
	if (_fd < 0) {
		// open fd
		//_fd = ::open(_port, O_RDWR | O_NOCTTY);
	}

	if (should_exit()) {
		VnSensor_disconnect(&_vs);
		exit_and_cleanup();
		return;

	} else if (!_initialized) {
		if (init() == PX4_OK) {
			_initialized = true;

		} else {
			PX4_ERR("init failed");
			exit_and_cleanup();
			return;
		}
	}

	ScheduleDelayed(100_ms);
}

int VectorNav::print_status()
{
	printf("Using port '%s'\n", _port);

	// if (_device[0] != '\0') {
	// 	PX4_INFO("UART device: %s", _device);
	// 	PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	// }

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	return 0;
}

int VectorNav::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return -1;
	}

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		VectorNav *instance = new VectorNav(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleNow();

		return PX4_OK;

	} else {
		if (device_name) {
			PX4_ERR("invalid device (-d) %s", device_name);

		} else {
			PX4_INFO("valid device required");
		}
	}

	return PX4_ERROR;
}

int VectorNav::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VectorNav::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the VectorNav VN-100, VN-200, VN-300.

Most boards are configured to enable/start the driver on a specified UART using the SENS_VN_CFG parameter.

Setup/usage information: https://docs.px4.io/master/en/sensor/vectornav.html

### Examples

Attempt to start driver on a specified serial device.
$ vectornav start -d /dev/ttyS1
Stop driver
$ vectornav stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vectornav", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");

	return PX4_OK;
}

extern "C" __EXPORT int vectornav_main(int argc, char *argv[])
{
	return VectorNav::main(argc, argv);
}
