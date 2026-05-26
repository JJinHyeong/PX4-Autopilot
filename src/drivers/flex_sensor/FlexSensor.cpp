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

#include "FlexSensor.hpp"
#include <px4_platform_common/log.h>

FlexSensor::FlexSensor() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_no_data_perf(perf_alloc(PC_COUNT,   MODULE_NAME": no_adc_data"))
{
}

FlexSensor::~FlexSensor()
{
	ScheduleClear();
	perf_free(_loop_perf);
	perf_free(_no_data_perf);
}

bool FlexSensor::init()
{
	static const char *flat_names[FLEX_NUM_CHANNELS] = {
		"FLEX_CAL_FLAT0", "FLEX_CAL_FLAT1", "FLEX_CAL_FLAT2", "FLEX_CAL_FLAT3"
	};
	static const char *deg90_names[FLEX_NUM_CHANNELS] = {
		"FLEX_CAL_90D0", "FLEX_CAL_90D1", "FLEX_CAL_90D2", "FLEX_CAL_90D3"
	};

	for (unsigned i = 0; i < FLEX_NUM_CHANNELS; i++) {
		_ph_flat[i]  = param_find(flat_names[i]);
		_ph_90deg[i] = param_find(deg90_names[i]);
	}

	update_params();
	ScheduleOnInterval(FLEX_POLL_INTERVAL_US);
	return true;
}

void FlexSensor::update_params()
{
	for (unsigned i = 0; i < FLEX_NUM_CHANNELS; i++) {
		if (_ph_flat[i] != PARAM_INVALID) {
			param_get(_ph_flat[i], &_raw_flat[i]);
		}

		if (_ph_90deg[i] != PARAM_INVALID) {
			param_get(_ph_90deg[i], &_raw_90deg[i]);
		}

		if (_raw_flat[i] == _raw_90deg[i]) {
			PX4_WARN("FLEX ch%u: CAL_FLAT == CAL_90DEG, resetting to defaults", i);
			_raw_flat[i]  = FLEX_CAL_FLAT_DEFAULT;
			_raw_90deg[i] = FLEX_CAL_90DEG_DEFAULT;
		}
	}
}

void FlexSensor::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	adc_report_s adc{};

	if (!_adc_sub.update(&adc)) {
		perf_count(_no_data_perf);
		perf_end(_loop_perf);
		return;
	}

	flex_sensor_s report{};
	report.timestamp = hrt_absolute_time();
	report.device_id = adc.device_id;

	for (unsigned i = 0; i < FLEX_NUM_CHANNELS; i++) {
		if (adc.channel_id[i] < 0) {
			continue;
		}

		const int32_t raw     = adc.raw_data[i];
		const float   voltage = (float)raw * (ADS1115_V_REF / (float)ADS1115_RESOLUTION);

		float bend_pct = (float)(_raw_flat[i] - raw) / (float)(_raw_flat[i] - _raw_90deg[i]) * 100.0f;

		if (bend_pct < 0.0f)   { bend_pct = 0.0f; }
		if (bend_pct > 100.0f) { bend_pct = 100.0f; }

		report.raw_adc[i]        = raw;
		report.voltage_v[i]      = voltage;
		report.bend_pct[i]       = bend_pct;
		report.flex_angle_deg[i] = bend_pct * 0.9f;

		_last_raw[i]      = raw;
		_last_voltage[i]  = voltage;
		_last_bend_pct[i] = bend_pct;
		_last_angle[i]    = bend_pct * 0.9f;
	}

	_pub.publish(report);

	perf_end(_loop_perf);
}

int FlexSensor::print_status()
{
	ModuleBase::print_status();
	PX4_INFO("ch    raw     volt(V)  bend%%   angle");
	PX4_INFO("----  ------  -------  ------  ------");

	for (unsigned i = 0; i < FLEX_NUM_CHANNELS; i++) {
		PX4_INFO("AIN%u  %6d  %6.3f   %5.1f%%  %5.1fdeg",
			 i, (int)_last_raw[i], (double)_last_voltage[i],
			 (double)_last_bend_pct[i], (double)_last_angle[i]);
	}

	return 0;
}

int FlexSensor::task_spawn(int argc, char *argv[])
{
	FlexSensor *instance = new FlexSensor();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return -1;
	}

	return 0;
}

int FlexSensor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlexSensor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Flex sensor module. Reads AIN0-AIN3 from the ads1115 driver via adc_report
and publishes bend angle and percentage for all 4 channels on the flex_sensor uORB topic.

Calibration parameters (per channel):
  FLEX_CAL_FLATx  – raw ADC at flat (0 deg),  default 11600
  FLEX_CAL_90Dx   – raw ADC at 90 degrees,     default 9180

### Examples
  flex_sensor start
  flex_sensor stop
  flex_sensor status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flex_sensor", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flex_sensor_main(int argc, char *argv[])
{
	return FlexSensor::main(argc, argv);
}
