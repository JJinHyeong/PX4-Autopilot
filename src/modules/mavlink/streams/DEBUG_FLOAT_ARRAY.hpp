/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef DEBUG_FLOAT_ARRAY_HPP
#define DEBUG_FLOAT_ARRAY_HPP

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/debug_array.h>

class MavlinkStreamDebugFloatArray : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDebugFloatArray(mavlink); }

	static constexpr const char *get_name_static() { return "DEBUG_FLOAT_ARRAY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _debug_array_subs.advertised_count() * (MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamDebugFloatArray(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<debug_array_s> _debug_array_subs{ORB_ID::debug_array};

	bool send() override
	{
		bool sent = false;

		for (int i = 0; i < _debug_array_subs.size(); i++) {
			debug_array_s debug;

			if (_debug_array_subs[i].update(&debug)) {
				mavlink_debug_float_array_t msg{};

				msg.time_usec = debug.timestamp;
				msg.array_id = debug.id;
				memcpy(msg.name, debug.name, sizeof(msg.name));
				msg.name[sizeof(msg.name) - 1] = '\0';

				for (size_t j = 0; j < debug_array_s::ARRAY_SIZE; j++) {
					msg.data[j] = debug.data[j];
				}

				mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);
				sent = true;
			}
		}

		return sent;
	}
};

#endif // DEBUG_FLOAT_ARRAY_HPP
