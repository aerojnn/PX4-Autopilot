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

#ifndef AIRDATA_BOOM_HPP
#define AIRDATA_BOOM_HPP

#include <uORB/topics/airdata_boom.h>
#include <uORB/topics/system_identification_data.h>

class MavlinkStreamAirdataBoom : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAirdataBoom(mavlink); }

	static constexpr const char *get_name_static() { return "AIRDATA_BOOM"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AIRDATA_BOOM; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _airdata_boom_sub.advertised() ? MAVLINK_MSG_ID_AIRDATA_BOOM_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamAirdataBoom(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _airdata_boom_sub{ORB_ID(airdata_boom)};
	uORB::Subscription _system_identification_data_sub{ORB_ID(system_identification_data)};

	bool send() override
	{
		// airdata_boom_s boom;
		system_identification_data_s sys_iden_data;  //debug testing

		if (_system_identification_data_sub.update(&sys_iden_data)) {
			mavlink_airdata_boom_t msg{};

			msg.indicated_airspeed = 0.0f;
			msg.calibrated_airspeed = 0.0f;
			msg.true_airspeed = sys_iden_data.servo;
			msg.aoa = sys_iden_data.d_a;
			msg.aos = sys_iden_data.d_e;
			msg.air_temperature = sys_iden_data.d_r;

			mavlink_msg_airdata_boom_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // AIRDATA_BOOM_HPP
