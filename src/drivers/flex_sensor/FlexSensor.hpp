/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/flex_sensor.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

/* ADS1115 constants – must match the ads1115 driver configuration */
#define ADS1115_V_REF       6.144f
#define ADS1115_RESOLUTION  32768

#define FLEX_NUM_CHANNELS       4
#define FLEX_CAL_FLAT_DEFAULT   11600
#define FLEX_CAL_90DEG_DEFAULT   9180

/* Poll at 50 Hz */
#define FLEX_POLL_INTERVAL_US   20000u

class FlexSensor : public ModuleBase<FlexSensor>, public px4::ScheduledWorkItem
{
public:
	FlexSensor();
	~FlexSensor() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void update_params();

	uORB::Subscription               _adc_sub{ORB_ID(adc_report)};
	uORB::Publication<flex_sensor_s> _pub{ORB_ID(flex_sensor)};

	perf_counter_t _loop_perf;
	perf_counter_t _no_data_perf;

	int32_t _raw_flat[FLEX_NUM_CHANNELS] {
		FLEX_CAL_FLAT_DEFAULT, FLEX_CAL_FLAT_DEFAULT,
		FLEX_CAL_FLAT_DEFAULT, FLEX_CAL_FLAT_DEFAULT
	};
	int32_t _raw_90deg[FLEX_NUM_CHANNELS] {
		FLEX_CAL_90DEG_DEFAULT, FLEX_CAL_90DEG_DEFAULT,
		FLEX_CAL_90DEG_DEFAULT, FLEX_CAL_90DEG_DEFAULT
	};

	param_t _ph_flat[FLEX_NUM_CHANNELS]  {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	param_t _ph_90deg[FLEX_NUM_CHANNELS] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};

	/* cached for print_status */
	int32_t _last_raw[FLEX_NUM_CHANNELS]      {};
	float   _last_voltage[FLEX_NUM_CHANNELS]  {};
	float   _last_bend_pct[FLEX_NUM_CHANNELS] {};
	float   _last_angle[FLEX_NUM_CHANNELS]    {};
};
