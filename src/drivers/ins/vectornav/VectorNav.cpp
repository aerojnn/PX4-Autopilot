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
	VnSensor_unregisterAsyncPacketReceivedHandler(&_vs);
	VnSensor_disconnect(&_vs);

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

void VectorNav::asciiOrBinaryAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex)
{
	if (VnUartPacket_isError(packet)) {
		uint8_t error = 0;
		VnUartPacket_parseError(packet, &error);

		char buffer[128] {};
		strFromSensorError(buffer, (SensorError)error);

		PX4_ERR("%s", buffer);

	} else if (userData && (VnUartPacket_type(packet) == PACKETTYPE_BINARY)) {
		static_cast<VectorNav *>(userData)->sensorCallback(packet);
	}
}

void VectorNav::sensorCallback(VnUartPacket *packet)
{
	const hrt_abstime time_now_us = hrt_absolute_time();

	BinaryGroupType groups = (BinaryGroupType)VnUartPacket_groups(packet);

	size_t curGroupFieldIndex = 0;

	// Output Group 1 Common Group
	if (groups & BINARYGROUPTYPE_COMMON) {

	}

	// Output Group 2 Time Group
	if (groups & BINARYGROUPTYPE_TIME) {

	}

	// Output Group 3 IMU Group
	if (groups & BINARYGROUPTYPE_IMU) {

		ImuGroup group_field = (ImuGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++);

		// 0: ImuStatus
		if (group_field & IMUGROUP_IMUSTATUS) {

		}

		// 4: Temp, 5: Pres
		if ((group_field & IMUGROUP_PRES) && (group_field & IMUGROUP_TEMP)) {

			const float temperature = VnUartPacket_extractFloat(packet);
			const float pressure = VnUartPacket_extractFloat(packet);

			sensor_baro_s sensor_baro{};
			sensor_baro.device_id = 0; // TODO: DRV_INS_DEVTYPE_VN300;
			sensor_baro.pressure = pressure;
			sensor_baro.temperature = temperature;
			sensor_baro.timestamp = hrt_absolute_time();

			_sensor_baro_pub.publish(sensor_baro);

			// update all temperatures
			_px4_accel.set_temperature(temperature);
			_px4_gyro.set_temperature(temperature);
			_px4_mag.set_temperature(temperature);
		}

		// 8: Mag
		if (group_field & IMUGROUP_MAG) {
			vec3f magnetic = VnUartPacket_extractVec3f(packet);
			_px4_mag.update(time_now_us, magnetic.c[0], magnetic.c[1], magnetic.c[2]);
		}

		// 9: Accel
		if (group_field & IMUGROUP_ACCEL) {
			vec3f acceleration = VnUartPacket_extractVec3f(packet);
			_px4_accel.update(time_now_us, acceleration.c[0], acceleration.c[1], acceleration.c[2]);
		}

		// 10: AngularRate
		if (group_field & IMUGROUP_ANGULARRATE) {
			vec3f angularRate = VnUartPacket_extractVec3f(packet);
			_px4_gyro.update(time_now_us, angularRate.c[0], angularRate.c[1], angularRate.c[2]);
		}
	}

	// TODO: temporary
	VnCompositeData cd{};
	VnCompositeData *compositeData = &cd;

	// Output Group 4 GNSS1 Group
	if (groups & BINARYGROUPTYPE_GPS) {

		GpsGroup group_field = (GpsGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++);

		sensor_gps_s sensor_gps{};

		// 0: UTC
		if (group_field & GPSGROUP_UTC) {

			sensor_gps.time_utc_usec = VnUartPacket_extractUint64(packet);

		}

		if (group_field & GPSGROUP_UTC) {
			compositeData->timeUtc.year = VnUartPacket_extractInt8(packet);
			compositeData->timeUtc.month = VnUartPacket_extractUint8(packet);
			compositeData->timeUtc.day = VnUartPacket_extractUint8(packet);
			compositeData->timeUtc.hour = VnUartPacket_extractUint8(packet);
			compositeData->timeUtc.min = VnUartPacket_extractUint8(packet);
			compositeData->timeUtc.sec = VnUartPacket_extractUint8(packet);
			compositeData->timeUtc.ms = VnUartPacket_extractUint16(packet);
		}

		if (group_field & GPSGROUP_TOW) {
			compositeData->gpsTow = VnUartPacket_extractUint64(packet);
		}

		if (group_field & GPSGROUP_WEEK) {
			compositeData->week = VnUartPacket_extractUint16(packet);
		}

		if (group_field & GPSGROUP_NUMSATS) {
			compositeData->numSats = VnUartPacket_extractUint8(packet);
		}

		if (group_field & GPSGROUP_FIX) {
			compositeData->gpsFix = VnUartPacket_extractUint8(packet);
		}

		if (group_field & GPSGROUP_POSLLA) {
			compositeData->positionGpsLla = VnUartPacket_extractVec3d(packet);
		}

		if (group_field & GPSGROUP_POSECEF) {
			compositeData->positionGpsEcef = VnUartPacket_extractVec3d(packet);
		}

		if (group_field & GPSGROUP_VELNED) {
			compositeData->velocityGpsNed = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & GPSGROUP_VELECEF) {
			compositeData->velocityGpsEcef = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & GPSGROUP_POSU) {
			compositeData->positionUncertaintyGpsNed = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & GPSGROUP_VELU) {
			compositeData->velocityUncertaintyGps = VnUartPacket_extractFloat(packet);
		}

		if (group_field & GPSGROUP_TIMEU) {
			compositeData->timeUncertainty = VnUartPacket_extractUint32(packet);
		}

		if (group_field & GPSGROUP_TIMEINFO) {
			compositeData->timeInfo = VnUartPacket_extractTimeInfo(packet);
		}

		if (group_field & GPSGROUP_DOP) {
			compositeData->dop = VnUartPacket_extractGpsDop(packet);
		}




		sensor_gps.timestamp = hrt_absolute_time();

		_sensor_gps_pub.publish(sensor_gps);
	}

	// Output Group 5 Attitude Group
	if (groups & BINARYGROUPTYPE_ATTITUDE) {

		AttitudeGroup group_field = (AttitudeGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++);

		if (group_field & ATTITUDEGROUP_QUATERNION) {

		}

		if (group_field & ATTITUDEGROUP_VPESTATUS) {
			compositeData->vpeStatus = VnUartPacket_extractUint16(packet);
		}

		if (group_field & ATTITUDEGROUP_YAWPITCHROLL) {
			compositeData->yawPitchRoll = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & ATTITUDEGROUP_QUATERNION) {
			compositeData->quaternion = VnUartPacket_extractVec4f(packet);
		}

		if (group_field & ATTITUDEGROUP_DCM) {
			compositeData->directionCosineMatrix = VnUartPacket_extractMat3f(packet);
		}

		if (group_field & ATTITUDEGROUP_MAGNED) {
			compositeData->magneticNed = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & ATTITUDEGROUP_ACCELNED) {
			compositeData->accelerationNed = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & ATTITUDEGROUP_LINEARACCELBODY) {
			compositeData->accelerationLinearBody = VnUartPacket_extractVec3f(packet);
			compositeData->velocityType = CDVEL_EstimatedBody;
		}

		if (group_field & ATTITUDEGROUP_LINEARACCELNED) {
			compositeData->accelerationLinearNed = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & ATTITUDEGROUP_YPRU) {
			compositeData->attitudeUncertainty = VnUartPacket_extractVec3f(packet);
		}

#ifdef EXTRA

		if (group_field & ATTITUDEGROUP_YPRRATE) {
			compositeData->yprRates = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & ATTITUDEGROUP_STATEAHRS) {
			/* Currently not doing anything with these values. */
			size_t i;

			for (i = 0; i < 7; i++) {
				VnUartPacket_extractFloat(packet);
			}
		}

		if (group_field & ATTITUDEGROUP_COVAHRS) {
			/* Currently not doing anything with these values. */
			size_t i;

			for (i = 0; i < 6; i++) {
				VnUartPacket_extractFloat(packet);
			}
		}

#endif



		//vehicle_attitude_s attitude{};

		// attitude.timestamp = hrt_absolute_time();
		//_attitude_pub.publish(attitude);
	}

	// Output Group 6 INS Group
	if (groups & BINARYGROUPTYPE_INS) {

		InsGroup group_field = (InsGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++);

		vehicle_attitude_s attitude{};
		vehicle_local_position_s local_position{};
		vehicle_global_position_s global_position{};

		if (group_field & INSGROUP_INSSTATUS) {
			compositeData->insStatus = VnUartPacket_extractUint16(packet);
			// Mode

			// GNSSFix

			// Error

			//
		}

		if (group_field & INSGROUP_POSLLA) {
			compositeData->positionEstimatedLla = VnUartPacket_extractVec3d(packet);
		}

		if (group_field & INSGROUP_POSECEF) {
			compositeData->positionEstimatedEcef = VnUartPacket_extractVec3d(packet);
		}

		if (group_field & INSGROUP_VELBODY) {
			compositeData->velocityEstimatedBody = VnUartPacket_extractVec3f(packet);
			compositeData->velocityType = CDVEL_EstimatedNed;
		}

		if (group_field & INSGROUP_VELNED) {
			compositeData->velocityEstimatedNed = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & INSGROUP_VELECEF) {
			compositeData->velocityEstimatedEcef = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & INSGROUP_MAGECEF) {
			compositeData->magneticEcef = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & INSGROUP_ACCELECEF) {
			compositeData->accelerationEcef = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & INSGROUP_LINEARACCELECEF) {
			compositeData->accelerationLinearEcef = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & INSGROUP_POSU) {
			compositeData->positionUncertaintyEstimated = VnUartPacket_extractFloat(packet);
		}

		if (group_field & INSGROUP_VELU) {
			compositeData->velocityUncertaintyEstimated = VnUartPacket_extractFloat(packet);
		}


		attitude.timestamp = hrt_absolute_time();
		_attitude_pub.publish(attitude);

		local_position.timestamp = hrt_absolute_time();
		_local_position_pub.publish(local_position);

		global_position.timestamp = hrt_absolute_time();
		_global_position_pub.publish(global_position);

	}

	// Output Group 7 GNSS2 Group
	if (groups & BINARYGROUPTYPE_GPS2) {

		GpsGroup group_field = (GpsGroup)VnUartPacket_groupField(packet, curGroupFieldIndex++);

		if (group_field & GPSGROUP_UTC) {
			/* TODO: Need to store this in the current data. */
			VnUartPacket_extractInt8(packet);
			VnUartPacket_extractUint8(packet);
			VnUartPacket_extractUint8(packet);
			VnUartPacket_extractUint8(packet);
			VnUartPacket_extractUint8(packet);
			VnUartPacket_extractUint8(packet);
			VnUartPacket_extractUint16(packet);
		}

		if (group_field & GPSGROUP_TOW) {
			compositeData->gps2Tow = VnUartPacket_extractUint64(packet);
		}

		if (group_field & GPSGROUP_WEEK) {
			compositeData->weekGps2 = VnUartPacket_extractUint16(packet);
		}

		if (group_field & GPSGROUP_NUMSATS) {
			compositeData->numSatsGps2 = VnUartPacket_extractUint8(packet);
		}

		if (group_field & GPSGROUP_FIX) {
			compositeData->fixGps2 = VnUartPacket_extractUint8(packet);
		}

		if (group_field & GPSGROUP_POSLLA) {
			compositeData->positionGps2Lla = VnUartPacket_extractVec3d(packet);
		}

		if (group_field & GPSGROUP_POSECEF) {
			compositeData->positionGps2Ecef = VnUartPacket_extractVec3d(packet);
		}

		if (group_field & GPSGROUP_VELNED) {
			compositeData->velocityGps2Ned = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & GPSGROUP_VELECEF) {
			compositeData->velocityGps2Ecef = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & GPSGROUP_POSU) {
			compositeData->positionUncertaintyGps2Ned = VnUartPacket_extractVec3f(packet);
		}

		if (group_field & GPSGROUP_VELU) {
			compositeData->velocityUncertaintyGps2 = VnUartPacket_extractFloat(packet);
		}

		if (group_field & GPSGROUP_TIMEU) {
			compositeData->timeUncertaintyGps2 = VnUartPacket_extractUint32(packet);
		}

#ifdef EXTRA

		if (gps2Group & GPSGROUP_SVSTAT) {
			/* Current not doing anything with these values. */
			size_t i;

			for (i = 0; i < 8; i++) {
				VnUartPacket_extractFloat(packet);
			}
		}

#endif

	}
}

bool VectorNav::init()
{
	// first try default baudrate
	const uint32_t DEFAULT_BAUDRATE = 115200;
	const uint32_t DESIRED_BAUDRATE = 921600;

	// first try default baudrate, if that fails try all other supported baudrates
	VnSensor_initialize(&_vs);

	if ((VnSensor_connect(&_vs, _port, DEFAULT_BAUDRATE) != E_NONE) || !VnSensor_verifySensorConnectivity(&_vs)) {

		static constexpr uint32_t BAUDRATES[] {9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600};

		for (auto &baudrate : BAUDRATES) {
			VnSensor_initialize(&_vs);

			if (VnSensor_connect(&_vs, _port, baudrate) == E_NONE && VnSensor_verifySensorConnectivity(&_vs)) {
				PX4_DEBUG("found baudrate %d", baudrate);
				break;
			}

			// disconnect before trying again
			VnSensor_disconnect(&_vs);
		}
	}

	VnError error = E_NONE;

	// change baudrate to max
	if ((error = VnSensor_changeBaudrate(&_vs, DESIRED_BAUDRATE)) != E_NONE) {
		PX4_WARN("Error changing baud rate failed: %d", error);
	}

	// query the sensor's model number
	char model_number[30] {};

	if ((error = VnSensor_readModelNumber(&_vs, model_number, sizeof(model_number))) != E_NONE) {
		PX4_ERR("Error reading model number %d", error);
		return false;
	}

	// query the sensor's hardware revision
	uint32_t hardware_revision = 0;

	if ((error = VnSensor_readHardwareRevision(&_vs, &hardware_revision)) != E_NONE) {
		PX4_ERR("Error reading HW revision %d", error);
		return false;
	}

	// query the sensor's serial number
	uint32_t serial_number = 0;

	if ((error = VnSensor_readSerialNumber(&_vs, &serial_number)) != E_NONE) {
		PX4_ERR("Error reading serial number %d", error);
		return false;
	}

	// query the sensor's firmware version
	char firmware_version[30] {};

	if ((error = VnSensor_readFirmwareVersion(&_vs, firmware_version, sizeof(firmware_version))) != E_NONE) {
		PX4_ERR("Error reading firmware version %d", error);
		return false;
	}

	PX4_INFO("Model: %s, HW REV: %d, SN: %d, SW VER: %s", model_number, hardware_revision, serial_number, firmware_version);

	return true;
}

bool VectorNav::configure()
{
	// disable all ASCII messages
	VnSensor_writeAsyncDataOutputType(&_vs, VNOFF, true);

	VnError error = E_NONE;

	/* For the registers that have more complex configuration options, it is
	 * convenient to read the current existing register configuration, change
	 * only the values of interest, and then write the configuration to the
	 * register. This allows preserving the current settings for the register's
	 * other fields. Below, we change the heading mode used by the sensor. */
	VpeBasicControlRegister vpeReg{};

	if (VnSensor_readVpeBasicControl(&_vs, &vpeReg) == E_NONE) {

		char strConversions[30] {};
		strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
		PX4_DEBUG("Old Heading Mode: %s\n", strConversions);

		vpeReg.enable = VNVPEENABLE_ENABLE;
		vpeReg.headingMode = VNHEADINGMODE_ABSOLUTE;

		if (VnSensor_writeVpeBasicControl(&_vs, vpeReg, true) == E_NONE) {

			if (VnSensor_readVpeBasicControl(&_vs, &vpeReg) == E_NONE) {
				strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
				PX4_DEBUG("New Heading Mode: %s\n", strConversions);
			}

		} else {
			PX4_ERR("Error writing VPE basic control");
		}

	} else {
		PX4_ERR("Error reading VPE basic control %d", error);
	}

	// 400 Hz
	BinaryOutputRegister_initialize(
		&_binary_output_400hz,
		ASYNCMODE_PORT1,
		1, // divider
		COMMONGROUP_NONE,
		TIMEGROUP_NONE,
		(ImuGroup)(IMUGROUP_ACCEL | IMUGROUP_ANGULARRATE),
		GPSGROUP_NONE,
		(AttitudeGroup)(ATTITUDEGROUP_QUATERNION | ATTITUDEGROUP_LINEARACCELNED),
		(InsGroup)(INSGROUP_INSSTATUS | INSGROUP_POSLLA | INSGROUP_VELNED | INSGROUP_POSU | INSGROUP_VELU),
		GPSGROUP_NONE
	);

	// 50 Hz (baro, mag)
	BinaryOutputRegister_initialize(
		&_binary_output_50hz,
		ASYNCMODE_PORT1,
		8, // divider
		COMMONGROUP_NONE,
		TIMEGROUP_NONE,
		(ImuGroup)(IMUGROUP_TEMP | IMUGROUP_PRES | IMUGROUP_MAG),
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
		GPSGROUP_NONE
	);

	// 5 Hz GPS
	// diviser 5 hz (diviser 80)
	BinaryOutputRegister_initialize(
		&_binary_output_5hz,
		ASYNCMODE_PORT1,
		80, // divider
		COMMONGROUP_NONE,
		TIMEGROUP_NONE,
		IMUGROUP_TEMP,
		(GpsGroup)(GPSGROUP_UTC | GPSGROUP_NUMSATS | GPSGROUP_FIX | GPSGROUP_POSLLA | GPSGROUP_VELNED | GPSGROUP_POSU |
			   GPSGROUP_VELU | GPSGROUP_DOP),
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
		GPSGROUP_NONE
	);


	if ((error = VnSensor_writeBinaryOutput1(&_vs, &_binary_output_400hz, true)) != E_NONE) {

		// char buffer[128]{};
		// strFromVnError((char*)buffer, error);
		// PX4_ERR("Error writing binary output 1: %s", buffer);
		PX4_ERR("Error writing binary output 1 %d", error);
		return false;
	}

	if ((error = VnSensor_writeBinaryOutput2(&_vs, &_binary_output_50hz, true)) != E_NONE) {
		PX4_ERR("Error writing binary output 2 %d", error);
		return false;
	}

	if ((error = VnSensor_writeBinaryOutput3(&_vs, &_binary_output_5hz, true)) != E_NONE) {
		PX4_ERR("Error writing binary output 3 %d", error);
		//return false;
	}

	VnSensor_registerAsyncPacketReceivedHandler(&_vs, VectorNav::asciiOrBinaryAsyncMessageReceived, this);


	// TODO:
	// VnSensor_registerErrorPacketReceivedHandler

	return true;
}

void VectorNav::Run()
{
	if (should_exit()) {
		VnSensor_unregisterAsyncPacketReceivedHandler(&_vs);
		VnSensor_disconnect(&_vs);
		exit_and_cleanup();
		return;

	} else if (!_initialized) {

		if (!_connected) {
			if (init()) {
				_connected = true;
			}
		}

		if (_connected) {
			if (!_configured) {
				if (configure()) {
					_configured = true;
				}
			}
		}

		if (_connected && _configured) {
			_initialized = true;

		} else {
			ScheduleDelayed(3_s);
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
