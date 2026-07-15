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

/**
 * @file FlexSensor.cpp
 *
 * ■ 센서 통신 프로토콜 요약
 * ─────────────────────────────────────────────────────────
 * [초기화 시퀀스]
 *   1. RESET  커맨드 전송 → 50ms 대기  (센서 내부 재초기화)
 *   2. GET_DEV_ID 전송   → 2ms 후 5바이트 읽기 (1축/2축 판별)
 *   3. SPS 커맨드 전송   → 100Hz 샘플레이트 설정
 *   4. ADS_RUN 활성화    → 연속 출력 모드 시작
 *
 * [데이터 읽기 - Free Run Mode]
 *   커맨드 전송 없이 5바이트 읽기:
 *   (센서가 100Hz로 지속 갱신, 읽을 때 최신 샘플 반환)
 *   1. 5바이트 읽기:
 *      buf[0] = 패킷 타입 (0x00 = 각도 데이터)
 *      buf[1] = Axis1 LSB  ┐ little-endian int16
 *      buf[2] = Axis1 MSB  ┘ → raw1 / 32.0 = 각도(°)
 *      buf[3] = Axis2 LSB  ┐
 *      buf[4] = Axis2 MSB  ┘ → raw2 / 32.0 = 각도(°)
 *
 * ■ Raw 값 의미
 * ─────────────────────────────────────────────────────────
 *   raw는 센서가 내부적으로 출력하는 int16 카운트 값
 *   전압이 아닌 정전용량을 디지털화한 값
 *   각도(°) = raw / 32.0  (공식 BendLabs 코드 기준)
 *   예) raw1 = 2880 → 2880 / 32.0 = 90.0°
 *       raw1 = -2880 → -90.0° (반대 방향)
 *       raw1 = 0     → 0° (파워온 기준 위치)
 *
 * ■ 상태(State) 분류 기준
 * ─────────────────────────────────────────────────────────
 *   STATE_FLAT  (0): 두 축 모두 3° 미만  → 평평한 상태
 *   STATE_BENT  (1): 한 축이 3° 이상     → 한 방향으로 굽힘
 *   STATE_TWIST (2): 두 축 모두 10° 초과 → 두 방향 동시 굽힘(뒤틀림)
 */

#include "FlexSensor.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

volatile uint8_t FlexSensor::_zero_mask = 0;
volatile bool    FlexSensor::_do_shutdown_test = false;

static inline float median3(float a, float b, float c)
{
	if (a > b) { float t = a; a = b; b = t; }
	if (b > c) { float t = b; b = c; c = t; }
	if (a > b) { float t = a; a = b; b = t; }
	return b;
}


FlexSensor::FlexSensor(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_err_perf(perf_alloc(PC_COUNT,   MODULE_NAME": i2c_err")),
	_is_2axis(config.custom1 == 1)   /* -2 플래그로 강제 2축 지정 가능 */
{
	_scale = _is_2axis ? ADS_SCALE_2AXIS : ADS_SCALE_1AXIS;
}

FlexSensor::~FlexSensor()
{
	/* 소멸 시 연속 출력 모드 정지 */
	uint8_t stop[ADS_TX_SIZE] = {ADS_CMD_RUN, 0x00};
	send_cmd(stop, sizeof(stop));

	perf_free(_loop_perf);
	perf_free(_err_perf);
}

/* ----------------------------------------------------------------
 * send_cmd : I²C 커맨드 전송 (쓰기 전용)
 * buf[0] = 커맨드 바이트, buf[1..] = 파라미터
 * ---------------------------------------------------------------- */
int FlexSensor::send_cmd(const uint8_t *buf, uint8_t len)
{
	int ret = transfer(buf, len, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_err_perf);
	}

	return ret;
}

/* ----------------------------------------------------------------
 * read_data : I²C 데이터 수신 (읽기 전용)
 * 커맨드 없이 센서에서 바로 len 바이트 읽기
 * ---------------------------------------------------------------- */
int FlexSensor::read_data(uint8_t *buf, uint8_t len)
{
	int ret = transfer(nullptr, 0, buf, len);

	if (ret != PX4_OK) {
		perf_count(_err_perf);
	}

	return ret;
}

/* ----------------------------------------------------------------
 * get_device_id : 센서 종류 식별
 *   커맨드 0x0A 전송 (5바이트 프레임) → 2ms 후 5바이트 응답
 *   응답 buf[0] = 0x02 (DEV_ID 패킷 타입)
 *   응답 buf[1] = 디바이스 ID
 *     1  → 1축 v1
 *     12 → 1축 v2
 *     2  → 2축 v1
 *     22 → 2축 v2 (Nitto)
 * ---------------------------------------------------------------- */
int FlexSensor::get_device_id(uint8_t &dev_id)
{
	uint8_t cmd[ADS_TX_SIZE] = {ADS_CMD_GET_DEV_ID};

	if (send_cmd(cmd, sizeof(cmd)) != PX4_OK) {
		return -EIO;
	}

	px4_usleep(2000);  /* 센서가 커맨드를 처리할 때까지 대기 */

	uint8_t buf[5] = {};  /* 공식 코드: ADS_TRANSFER_SIZE=5바이트 읽기 */

	if (read_data(buf, sizeof(buf)) != PX4_OK) {
		return -EIO;
	}

	PX4_INFO("[0x%02X] DEV_ID raw: buf=[0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]",
		 get_device_address(), buf[0], buf[1], buf[2], buf[3], buf[4]);

	if (buf[0] != ADS_PKT_DEV_ID) {
		PX4_WARN("[0x%02X] buf[0]=0x%02X != 0x02 (SAMPLE 패킷으로 응답 - 타이밍 문제)",
			 get_device_address(), buf[0]);
	} else {
		PX4_INFO("[0x%02X] buf[0]=0x02 OK, dev_id=%u", get_device_address(), buf[1]);
	}

	dev_id = buf[1];
	return PX4_OK;
}

/* ----------------------------------------------------------------
 * init : 센서 초기화 시퀀스
 *   I2C 버스 초기화 → RESET → GET_DEV_ID → SPS 설정 → RUN 활성화
 * ---------------------------------------------------------------- */
int FlexSensor::init()
{
	/* 1단계: I²C 버스 및 슬레이브 주소 초기화 */
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed (%d) bus %d addr 0x%02X",
			ret, get_device_bus(), get_device_address());
		return ret;
	}

	uint8_t rst[ADS_TX_SIZE] = {ADS_CMD_RESET};

	if (send_cmd(rst, sizeof(rst)) != PX4_OK) {
		PX4_ERR("RESET failed bus %d addr 0x%02X",
			get_device_bus(), get_device_address());
		return -EIO;
	}

	px4_usleep(50000);

	if (get_device_id(_dev_id) == PX4_OK) {
		switch (_dev_id) {
		case ADS_DEV_1AXIS:
		case ADS_DEV_1AXIS_V2:
			if (!_is_2axis) { _scale = ADS_SCALE_1AXIS; }

			PX4_INFO("1-axis sensor detected (dev_id=%u)", _dev_id);
			break;

		case ADS_DEV_2AXIS:
		case ADS_DEV_2AXIS_V2:
			_is_2axis = true;
			_scale    = ADS_SCALE_2AXIS;
			PX4_INFO("2-axis sensor detected (dev_id=%u)", _dev_id);
			break;

		default:
			PX4_WARN("unknown dev_id=%u, using %s mode", _dev_id, _is_2axis ? "2-axis" : "1-axis");
			break;
		}

	} else {
		PX4_WARN("DEV_ID read failed, using %s mode", _is_2axis ? "2-axis" : "1-axis");
	}

	uint8_t sps[ADS_TX_SIZE] = {
		ADS_CMD_SPS,
		(uint8_t)(ADS_SPS_100HZ & 0xFF),
		(uint8_t)((ADS_SPS_100HZ >> 8) & 0xFF)
	};

	if (send_cmd(sps, sizeof(sps)) != PX4_OK) {
		PX4_WARN("SPS set failed");
	}

	px4_usleep(5000);

	uint8_t run_en[ADS_TX_SIZE] = {ADS_CMD_RUN, 0x01};

	if (send_cmd(run_en, sizeof(run_en)) != PX4_OK) {
		PX4_ERR("RUN mode failed");
		return -EIO;
	}

	px4_usleep(5000);

	PX4_INFO("init OK bus %d addr 0x%02X %s 100Hz",
		 get_device_bus(), get_device_address(),
		 _is_2axis ? "2-axis" : "1-axis");

	_sensor_idx = (int)(get_device_address() - ADS_DEFAULT_ADDR);

	if (_sensor_idx < 0 || _sensor_idx > 3) { _sensor_idx = 0; }

	char pname[16];
	snprintf(pname, sizeof(pname), "FLEX_S%d_A1_OFF", _sensor_idx + 1);
	_param_axis1_off = param_find(pname);
	snprintf(pname, sizeof(pname), "FLEX_S%d_A2_OFF", _sensor_idx + 1);
	_param_axis2_off = param_find(pname);

	/* Auto-zero on startup: sensor is in a fixed mount so boot position is
	 * always the neutral reference. Zeroed on the first RunImpl call. */
	_zero_mask |= (uint8_t)(1u << _sensor_idx);

	ScheduleOnInterval(ADS_POLL_US);  /* 10ms 주기로 RunImpl 호출 (100Hz) */
	return PX4_OK;
}

/* ----------------------------------------------------------------
 * RunImpl : 10ms(100Hz)마다 호출되는 데이터 수집 루틴
 *
 * [동작 순서]
 *   1. 5바이트 읽기      → [패킷타입 | A1_L A1_H | A2_L A2_H]
 *      (연속 출력 모드: 센서가 100Hz로 계속 갱신, 별도 요청 불필요)
 *   2. 파싱 및 변환      → raw / 32.0 = 각도(°)
 *   3. 합성값 계산       → total, direction
 *   4. 상태 분류         → FLAT / BENT / TWIST
 *   5. uORB 발행         → flex_sensor 토픽
 * ---------------------------------------------------------------- */
void FlexSensor::RunImpl()
{
	perf_begin(_loop_perf);

	/* SHUTDOWN 복구 테스트 요청 시 일회성 실행 (RunImpl 독점) */
	if (_do_shutdown_test) {
		_do_shutdown_test = false;
		run_shutdown_test();
		perf_end(_loop_perf);
		return;
	}

	/* ── Step 1: 5바이트 응답 읽기 ── */
	uint8_t buf[5] = {};
	const uint8_t rlen = _is_2axis ? 5u : 3u;

	if (read_data(buf, rlen) != PX4_OK) {
		perf_end(_loop_perf);
		return;
	}

	/* 디버그: raw 바이트 출력 (문제 진단용) */
	PX4_DEBUG("raw[%02X %02X %02X %02X %02X]",
		  buf[0], buf[1], buf[2], buf[3], buf[4]);

	/* ── Step 2: 패킷 타입 확인 ──
	 * buf[0] == 0x00 이어야 각도 데이터 패킷 */
	if (buf[0] != ADS_PKT_SAMPLE) {
		PX4_WARN("unexpected pkt type 0x%02X", buf[0]);
		perf_end(_loop_perf);
		return;
	}

	/* ── Step 3: raw 카운트 파싱 (little-endian int16) ──
	 *
	 * 바이트 레이아웃:
	 *   buf[1]=LSB, buf[2]=MSB → Axis1
	 *   buf[3]=LSB, buf[4]=MSB → Axis2
	 *
	 * 예) buf = [0x00, 0xC0, 0x0A, 0x20, 0xFF]
	 *   raw1 = 0x0AC0 = 2752 → 2752 / 32.0 = 86.0°
	 *   raw2 = 0xFF20 = -224 → -224 / 32.0 = -7.0°
	 */
	int16_t raw1 = (int16_t)((uint16_t)buf[2] << 8 | buf[1]);
	int16_t raw2 = _is_2axis ? (int16_t)((uint16_t)buf[4] << 8 | buf[3]) : 0;

	float axis1 = (float)raw1 * _scale;  /* 각도 변환: raw / 32.0 (= raw × 0.03125°/LSB) */
	float axis2 = (float)raw2 * _scale;

	/* 물리적으로 불가능한 값 즉시 폐기 (센서 최대 ±105°, 여유 5° 포함) */
	if (fabsf(axis1) > ADS_RANGE_DEG || fabsf(axis2) > ADS_RANGE_DEG) {
		perf_count(_err_perf);
		perf_end(_loop_perf);
		return;
	}

	/* ── zero 명령 처리 (flex_sensor zero 로 요청됨) ──
	 * 현재 raw 각도를 오프셋으로 저장 → 이후부터 이 자세가 0° 기준 */
	uint8_t bit = (uint8_t)(1u << _sensor_idx);

	if (_zero_mask & bit) {
		_zero_mask &= ~bit;
		param_set(_param_axis1_off, &axis1);
		param_set(_param_axis2_off, &axis2);
		/* Reset filter state so the median buffer reinitializes from ~0 deg
		 * instead of carrying over pre-zero values. */
		_last_axis1 = 0.0f;
		_last_axis2 = 0.0f;
		for (int k = 0; k < ADS_MEDIAN_N; k++) { _hist1[k] = 0.0f; _hist2[k] = 0.0f; }
		_hist_idx   = 0;
		_read_count = 0;
		PX4_INFO("Sensor %d [0x%02X] zeroed: A1=%.2f A2=%.2f deg saved",
			 _sensor_idx + 1, get_device_address(),
			 (double)axis1, (double)axis2);
	}

	/* ── Apply zero offset (FLEX_SN_A1_OFF / FLEX_SN_A2_OFF params) ── */
	float off1 = 0.0f, off2 = 0.0f;
	param_get(_param_axis1_off, &off1);
	param_get(_param_axis2_off, &off2);
	axis1 -= off1;
	axis2 -= off2;

	/* ── Median Filter N=3 ──────────────────────────────────────────
	 * Circular buffer of last 3 samples. Returns middle value.
	 * Removes single-sample I2C spikes with only 2-sample (20ms) lag. */
	_hist1[_hist_idx] = axis1;
	_hist2[_hist_idx] = axis2;
	_hist_idx = (_hist_idx + 1) % ADS_MEDIAN_N;

	if (_read_count >= (ADS_MEDIAN_N - 1)) {
		axis1 = median3(_hist1[0], _hist1[1], _hist1[2]);
		axis2 = median3(_hist2[0], _hist2[1], _hist2[2]);
	}

	/* ── Step 4: 합성값 계산 ──
	 *
	 * total     : 두 축의 벡터 합성 굽힘각
	 *             센서가 어느 방향으로 얼마나 굽었는지 나타내는 크기
	 *
	 * direction : 굽힘 방향 (도 단위)
	 *             0°  = Axis1 방향으로만 굽힘
	 *             90° = Axis2 방향으로만 굽힘
	 *             45° = 두 축 동일하게 굽힘
	 */
	const float total = sqrtf(axis1 * axis1 + axis2 * axis2);
	const float dir   = atan2f(axis2, axis1) * (180.0f / M_PI_F);

	/* 캐시 업데이트 (status 출력용) */
	_last_raw1  = raw1;
	_last_raw2  = raw2;
	_last_axis1 = axis1;
	_last_axis2 = axis2;
	_last_total = total;
	_last_dir   = dir;
	_read_count++;

	/* ── Step 5: 상태 분류 ──
	 *
	 * FLAT  (0): 완전히 평평한 상태
	 *            두 축 모두 ADS_FLAT_THR_DEG(3°) 미만
	 *
	 * BENT  (1): 한 방향으로 굽힘
	 *            한 축이 3° 이상이지만 두 축 모두 10° 초과는 아님
	 *
	 * TWIST (2): 두 방향 동시 굽힘 / 뒤틀림
	 *            두 축 모두 ADS_TWIST_THR_DEG(10°) 초과
	 */
	/* ── Step 6: uORB 발행 ── */
	flex_sensor_s report{};
	report.timestamp     = hrt_absolute_time();
	report.axis1_deg     = axis1;
	report.axis2_deg     = axis2;
	report.total_deg     = total;
	report.direction_deg = dir;
	report.raw_axis1     = raw1;
	report.raw_axis2     = raw2;
	_pub.publish(report);

	/* MAVLink DEBUG_FLOAT_ARRAY 로 PC에 스트리밍
	 * id = I2C 주소 (0x13=19, 0x14=20, 0x15=21, 0x16=22) 로 센서 구분 */
	debug_array_s dbg{};
	dbg.timestamp = report.timestamp;
	dbg.id        = (uint16_t)get_device_address();
	dbg.data[0]   = axis1;
	dbg.data[1]   = axis2;
	dbg.data[2]   = total;
	dbg.data[3]   = dir;
	_debug_pub.publish(dbg);

	perf_end(_loop_perf);
}

/* ----------------------------------------------------------------
 * diag : I²C 버스 진단 루틴
 *
 * 1. 0x13~0x16 주소 스캔 (ACK 여부)
 * 2. 발견된 각 주소에서 단계별 init 시퀀스 (각 단계 성공/실패 출력)
 * 3. 20샘플 연속 읽기 (raw 바이트 + 각도 출력, freeze 판정)
 *
 * 사용: flex_sensor diag -X -b <bus>
 *   버스를 여기서 지정하면 스캔 결과로 문제 원인을 좁힐 수 있다
 * ---------------------------------------------------------------- */
void FlexSensor::diag()
{
	static const uint8_t SCAN_ADDRS[4] = {0x13, 0x14, 0x15, 0x16};

	PX4_INFO("════════════════════════════════════════════════════════");
	PX4_INFO("  ADS Flex Sensor I2C Diagnostic  (bus %d)", get_device_bus());
	PX4_INFO("════════════════════════════════════════════════════════");

	/* ── 1. 주소 스캔 ── */
	PX4_INFO("");
	PX4_INFO("[SCAN] Probing 0x13 0x14 0x15 0x16 ...");
	bool found[4] = {};

	for (int i = 0; i < 4; i++) {
		set_device_address(SCAN_ADDRS[i]);
		uint8_t probe = 0;
		found[i] = (transfer(nullptr, 0, &probe, 1) == PX4_OK);
		PX4_INFO("  0x%02X : %s", SCAN_ADDRS[i], found[i] ? "ACK  <-- found" : "NACK");
	}

	int total_found = 0;

	for (int i = 0; i < 4; i++) { if (found[i]) { total_found++; } }

	PX4_INFO("  Found %d device(s) on bus %d", total_found, get_device_bus());

	if (total_found == 0) {
		PX4_ERR("");
		PX4_ERR("[DIAG] Nothing found. Check:");
		PX4_ERR("  1) Wiring - SDA/SCL/GND/3.3V");
		PX4_ERR("  2) Bus number: try -b 1  -b 2  -b 3  -b 4");
		PX4_ERR("  3) External bus flag: add -X for external connector");
		PX4_ERR("  4) Pull-up resistors (4.7kOhm to 3.3V)");
		PX4_ERR("  5) Power: sensor needs 3.3V (NOT 5V)");
		return;
	}

	/* ── 2. 발견된 주소별 단계별 init ── */
	for (int i = 0; i < 4; i++) {
		if (!found[i]) { continue; }

		PX4_INFO("");
		PX4_INFO("[TEST] addr 0x%02X ─────────────────────────────────────", SCAN_ADDRS[i]);
		set_device_address(SCAN_ADDRS[i]);

		/* Step 1: RESET */
		uint8_t rst[ADS_TX_SIZE] = {ADS_CMD_RESET};
		bool rst_ok = (send_cmd(rst, sizeof(rst)) == PX4_OK);
		PX4_INFO("  [1/5] RESET (0x02)        : %s", rst_ok ? "OK  (+50ms wait)" : "FAIL -- sensor not responding");

		if (!rst_ok) {
			PX4_WARN("        Scan passed but RESET failed -> pull-up issue or power glitch");
			continue;
		}

		px4_usleep(50000);

		/* Step 2: GET_DEV_ID */
		uint8_t id_cmd[ADS_TX_SIZE] = {ADS_CMD_GET_DEV_ID};
		bool id_cmd_ok = (send_cmd(id_cmd, sizeof(id_cmd)) == PX4_OK);
		px4_usleep(2000);
		uint8_t id_buf[5] = {};
		bool id_rd_ok = (read_data(id_buf, sizeof(id_buf)) == PX4_OK);
		const char *dev_name = "unknown";

		if (id_rd_ok) {
			switch (id_buf[1]) {
			case  1: dev_name = "1-axis v1";         break;
			case 12: dev_name = "1-axis v2";         break;
			case  2: dev_name = "2-axis v1";         break;
			case 22: dev_name = "2-axis v2 (Nitto)"; break;
			}
		}

		PX4_INFO("  [2/5] GET_DEV_ID (0x0A)   : cmd=%s rd=%s  pkt=0x%02X dev_id=%u -> %s",
			 id_cmd_ok ? "OK" : "FAIL", id_rd_ok ? "OK" : "FAIL",
			 id_buf[0], id_buf[1], dev_name);

		if (id_rd_ok && id_buf[0] != ADS_PKT_DEV_ID) {
			PX4_WARN("        pkt=0x%02X expected 0x02 -- sensor already in streaming mode or timing off",
				 id_buf[0]);
		}

		/* Step 3: SPS 100Hz */
		uint8_t sps[ADS_TX_SIZE] = {
			ADS_CMD_SPS,
			(uint8_t)(ADS_SPS_100HZ & 0xFF),
			(uint8_t)((ADS_SPS_100HZ >> 8) & 0xFF)
		};
		bool sps_ok = (send_cmd(sps, sizeof(sps)) == PX4_OK);
		PX4_INFO("  [3/5] SPS 100Hz (0x01)    : %s", sps_ok ? "OK" : "FAIL");

		/* Step 4: RUN ON */
		uint8_t run[ADS_TX_SIZE] = {ADS_CMD_RUN, 0x01};
		bool run_ok = (send_cmd(run, sizeof(run)) == PX4_OK);
		PX4_INFO("  [4/5] RUN ON (0x00, 0x01) : %s  (+10ms for first sample)", run_ok ? "OK" : "FAIL");
		px4_usleep(10000);

		/* Step 5: 20샘플 연속 읽기 */
		PX4_INFO("  [5/5] Continuous read (20 samples @ 10ms):");

		int good = 0, bad_pkt = 0, rd_err = 0;
		int16_t prev_r1 = 0, prev_r2 = 0;
		bool data_frozen = true;

		for (int s = 0; s < 20; s++) {
			px4_usleep(10000);
			uint8_t buf[5] = {};

			if (read_data(buf, sizeof(buf)) != PX4_OK) {
				PX4_INFO("         [%2d] READ FAIL", s);
				rd_err++;
				continue;
			}

			int16_t r1 = (int16_t)((uint16_t)buf[2] << 8 | buf[1]);
			int16_t r2 = (int16_t)((uint16_t)buf[4] << 8 | buf[3]);

			if (s > 0 && (r1 != prev_r1 || r2 != prev_r2)) { data_frozen = false; }

			prev_r1 = r1;
			prev_r2 = r2;

			if (buf[0] == ADS_PKT_SAMPLE) {
				float a1 = (float)r1 * ADS_SCALE_2AXIS;
				float a2 = (float)r2 * ADS_SCALE_2AXIS;
				PX4_INFO("         [%2d] pkt=0x00 raw1=%6d raw2=%6d  A1=%7.2f  A2=%7.2f deg",
					 s, r1, r2, (double)a1, (double)a2);
				good++;

			} else {
				PX4_WARN("         [%2d] pkt=0x%02X (not 0x00)  raw=[%02X %02X %02X %02X]",
					 s, buf[0], buf[1], buf[2], buf[3], buf[4]);
				bad_pkt++;
			}
		}

		/* ── 결과 요약 ── */
		PX4_INFO("");
		PX4_INFO("  ── Result 0x%02X ──────────────────────────────────", SCAN_ADDRS[i]);
		PX4_INFO("  good=%d  bad_pkt=%d  read_err=%d", good, bad_pkt, rd_err);

		/* 물리 범위 초과 여부 확인 (±105° 기준) */
		static constexpr float PHYS_LIMIT = 105.0f;
		bool axis1_oor = fabsf((float)prev_r1 * ADS_SCALE_2AXIS) > PHYS_LIMIT;
		bool axis2_oor = fabsf((float)prev_r2 * ADS_SCALE_2AXIS) > PHYS_LIMIT;

		if (rd_err == 20) {
			PX4_ERR("  FAIL: all reads failed after successful init");
			PX4_ERR("        -> sensor entered auto-sleep?");
			PX4_ERR("        -> try: flex_sensor start, then flex_sensor status");

		} else if (good == 0) {
			PX4_ERR("  FAIL: no valid samples (pkt type always wrong)");
			PX4_ERR("        -> sensor firmware version mismatch?");

		} else if (data_frozen && good >= 5) {
			PX4_WARN("  WARNING: data frozen (values never changed)");
			PX4_WARN("           -> SPS during streaming freezes ADC on some units");
			PX4_WARN("           -> workaround: RESET -> RUN only (skip SPS)");

		} else {
			PX4_INFO("  PASS: sensor responding correctly");
		}

		if (axis1_oor) {
			PX4_WARN("  WARNING: Axis1 = %.1f deg exceeds physical limit (+-105 deg)",
				 (double)((float)prev_r1 * ADS_SCALE_2AXIS));
			PX4_WARN("           -> RunImpl range check WILL reject all samples");
			PX4_WARN("           -> Possible cause: sensor overstressed or Axis1 element damaged");
		}

		if (axis2_oor) {
			PX4_WARN("  WARNING: Axis2 = %.1f deg exceeds physical limit (+-105 deg)",
				 (double)((float)prev_r2 * ADS_SCALE_2AXIS));
		}
	}

	PX4_INFO("");
	PX4_INFO("════════════════════════════════════════════════════════");
	PX4_INFO("Next steps:");
	PX4_INFO("  Working  -> flex_sensor start -X -b <bus> -a <addr>");
	PX4_INFO("  Nothing  -> check bus number (try -b 1 through -b 4)");
	PX4_INFO("  Frozen   -> rebuild without SPS cmd in init");
	PX4_INFO("════════════════════════════════════════════════════════");
}

/* ----------------------------------------------------------------
 * set_device_addr : 센서 I²C 주소를 영구적으로 변경
 *
 * ADS_SET_ADDRESS(0x04) 커맨드를 센서에 전송하면 센서 내부 플래시에
 * 새 주소가 저장되어 전원을 꺼도 유지된다.
 *
 * [사용 방법]
 *   1. 변경할 센서 1개만 버스에 연결
 *   2. flex_sensor set_addr -X -b 3 -a 0x13 0x14  실행
 *   3. 전원을 껐다 켜서 새 주소(0x14)로 응답하는지 i2cdetect로 확인
 * ---------------------------------------------------------------- */
int FlexSensor::set_device_addr(uint8_t new_addr)
{
	uint8_t cmd[ADS_TX_SIZE] = {ADS_CMD_SET_ADDR, new_addr};
	return send_cmd(cmd, sizeof(cmd));
}

/* ----------------------------------------------------------------
 * factory_reset : 사용자 캘리브레이션 삭제 + 공장 캘리브레이션 복원
 *
 * ADS_CMD_CALIBRATE(0x07) + step=ADS_CAL_FACTORY_RESET(0x03) 전송.
 * 초기화 후에도 평평한 자세에서 raw 값이 비정상적으로 크게 나오면
 * (다른 정상 센서 대비 큰 오프셋) 사용자 캘리브레이션 문제가 아니라
 * 하드웨어 손상(과도한 스트레인 등)일 가능성이 높다는 진단 기준으로
 * 쓸 수 있다.
 * ---------------------------------------------------------------- */
int FlexSensor::factory_reset()
{
	uint8_t cmd[ADS_TX_SIZE] = {ADS_CMD_CALIBRATE, ADS_CAL_FACTORY_RESET};
	return send_cmd(cmd, sizeof(cmd));
}

/* ----------------------------------------------------------------
 * run_shutdown_test : SHUTDOWN(0x09)이 ADC freeze를 해제하는지 검증
 *
 * 테스트 순서:
 *   1. 기준 5샘플  → 정상 스트리밍 확인 (값이 변해야 함)
 *   2. SPS 재전송  → 스트리밍 중 SPS = ADC freeze 유발
 *   3. 확인 5샘플  → freeze 확인 (값이 고정되어야 함)
 *   4. SHUTDOWN    → 200ms 대기 (초저전력 모드)
 *   5. RESET       → 100ms 대기 (NVRAM 복원 후 재시작)
 *   6. 복구 5샘플  → 값이 다시 변하면 SHUTDOWN = 전원차단 동급
 *   7. 재초기화    → 드라이버 정상 복귀
 * ---------------------------------------------------------------- */
void FlexSensor::run_shutdown_test()
{
	PX4_INFO("====================================================");
	PX4_INFO("ADS SHUTDOWN Freeze-Recovery Test [addr 0x%02X]", get_device_address());
	PX4_INFO("====================================================");

	/* Step 1: 기준 5샘플 */
	PX4_INFO("[1] Baseline (sensor should be streaming, values must vary):");
	int16_t b1[5] = {}, b2[5] = {};

	for (int i = 0; i < 5; i++) {
		px4_usleep(15000);
		uint8_t buf[5] = {};
		read_data(buf, sizeof(buf));
		b1[i] = (int16_t)((uint16_t)buf[2] << 8 | buf[1]);
		b2[i] = (int16_t)((uint16_t)buf[4] << 8 | buf[3]);
		PX4_INFO("    [%d] raw1=%-6d  raw2=%-6d  pkt=0x%02X", i, b1[i], b2[i], buf[0]);
	}

	bool base_constant = true;

	for (int i = 1; i < 5; i++) {
		if (b1[i] != b1[0] || b2[i] != b2[0]) { base_constant = false; break; }
	}

	PX4_INFO("    -> %s", base_constant ? "CONSTANT (sensor may already be frozen!)" : "VARYING (normal)");

	if (base_constant) {
		PX4_WARN("Cannot induce freeze on already-frozen sensor. Aborting test.");
		return;
	}

	/* Step 2: SPS 재전송으로 ADC freeze 유발 */
	PX4_INFO("[2] Sending SPS(100Hz) while streaming -> inducing ADC freeze...");
	uint8_t sps[ADS_TX_SIZE] = {
		ADS_CMD_SPS,
		(uint8_t)(ADS_SPS_100HZ & 0xFF),
		(uint8_t)((ADS_SPS_100HZ >> 8) & 0xFF)
	};
	send_cmd(sps, sizeof(sps));
	px4_usleep(50000);

	/* Step 3: freeze 확인 */
	PX4_INFO("[3] Confirming freeze (all 5 samples must be identical):");
	int16_t f1[5] = {}, f2[5] = {};

	for (int i = 0; i < 5; i++) {
		px4_usleep(15000);
		uint8_t buf[5] = {};
		read_data(buf, sizeof(buf));
		f1[i] = (int16_t)((uint16_t)buf[2] << 8 | buf[1]);
		f2[i] = (int16_t)((uint16_t)buf[4] << 8 | buf[3]);
		PX4_INFO("    [%d] raw1=%-6d  raw2=%-6d", i, f1[i], f2[i]);
	}

	bool is_frozen = true;

	for (int i = 1; i < 5; i++) {
		if (f1[i] != f1[0] || f2[i] != f2[0]) { is_frozen = false; break; }
	}

	PX4_INFO("    -> %s", is_frozen ? "FROZEN (confirmed)" : "NOT FROZEN (test inconclusive)");

	if (!is_frozen) {
		PX4_WARN("Could not induce freeze. Possible causes:");
		PX4_WARN("  - Sensor was power-cycled (started in IDLE, SPS in IDLE is safe)");
		PX4_WARN("  - This Pixhawk test cannot simulate VOXL2 scenario unless sensor stays powered");
		/* 재초기화 */
		uint8_t idle[ADS_TX_SIZE] = {ADS_CMD_RUN, 0x00};
		send_cmd(idle, sizeof(idle));
		px4_usleep(50000);
		send_cmd(sps, sizeof(sps));
		px4_usleep(20000);
		uint8_t run[ADS_TX_SIZE] = {ADS_CMD_RUN, 0x01};
		send_cmd(run, sizeof(run));
		return;
	}

	/* Step 4: SHUTDOWN */
	PX4_INFO("[4] Sending SHUTDOWN (0x09)... waiting 200ms in ultra-low-power");
	uint8_t shutdown_cmd[ADS_TX_SIZE] = {ADS_CMD_SHUTDOWN};
	send_cmd(shutdown_cmd, sizeof(shutdown_cmd));
	px4_usleep(200000);

	/* Step 5: RESET으로 SHUTDOWN에서 깨우기 */
	PX4_INFO("[5] Sending RESET to wake from SHUTDOWN... waiting 100ms");
	uint8_t rst[ADS_TX_SIZE] = {ADS_CMD_RESET};
	send_cmd(rst, sizeof(rst));
	px4_usleep(100000);

	/* Step 6: 복구 확인 */
	PX4_INFO("[6] Sampling after SHUTDOWN+RESET:");
	int16_t r1[5] = {}, r2[5] = {};

	for (int i = 0; i < 5; i++) {
		px4_usleep(15000);
		uint8_t buf[5] = {};
		read_data(buf, sizeof(buf));
		r1[i] = (int16_t)((uint16_t)buf[2] << 8 | buf[1]);
		r2[i] = (int16_t)((uint16_t)buf[4] << 8 | buf[3]);
		PX4_INFO("    [%d] raw1=%-6d  raw2=%-6d  pkt=0x%02X", i, r1[i], r2[i], buf[0]);
	}

	bool still_frozen = true;

	for (int i = 1; i < 5; i++) {
		if (r1[i] != r1[0] || r2[i] != r2[0]) { still_frozen = false; break; }
	}

	bool recovered = !still_frozen;

	/* Step 7: 재초기화 */
	PX4_INFO("[7] Re-initializing sensor after test...");
	uint8_t idle[ADS_TX_SIZE] = {ADS_CMD_RUN, 0x00};
	send_cmd(idle, sizeof(idle));
	px4_usleep(50000);
	send_cmd(sps, sizeof(sps));
	px4_usleep(20000);
	uint8_t run[ADS_TX_SIZE] = {ADS_CMD_RUN, 0x01};
	send_cmd(run, sizeof(run));
	px4_usleep(20000);

	PX4_INFO("====================================================");
	PX4_INFO("RESULT:");
	PX4_INFO("  Freeze induced        : YES");
	PX4_INFO("  After SHUTDOWN+RESET  : %s", recovered ? "RECOVERED (values vary)" : "STILL FROZEN");

	if (recovered) {
		PX4_INFO("  >> CONCLUSION: SHUTDOWN IS equivalent to power cycle");
		PX4_INFO("     VOXL2 init fix: SHUTDOWN(200ms) -> RESET(100ms) -> IDLE -> SPS -> RUN");
	} else {
		PX4_INFO("  >> CONCLUSION: SHUTDOWN does NOT clear ADC freeze");
		PX4_INFO("     Only physical power cycle can recover. Hardware nRST line needed.");
		PX4_WARN("  Sensor [0x%02X] remains frozen - power cycle required", get_device_address());
	}

	PX4_INFO("====================================================");
}

/* ----------------------------------------------------------------
 * print_status : 'flex_sensor status' 명령 출력
 * ---------------------------------------------------------------- */
void FlexSensor::print_status()
{
	I2CSPIDriverBase::print_status();
	const int sensor_num = (int)(get_device_address() - ADS_DEFAULT_ADDR) + 1;
	PX4_INFO("Sensor %d [0x%02X] ----------", sensor_num, get_device_address());
	PX4_INFO("  type      : %s (dev_id=%u)", _is_2axis ? "2-axis" : "1-axis", _dev_id);
	PX4_INFO("  reads     : %u", (unsigned)_read_count);
	PX4_INFO("  Axis1     : %7.2f deg (raw=%d)", (double)_last_axis1, (int)_last_raw1);
	PX4_INFO("  Axis2     : %7.2f deg (raw=%d)", (double)_last_axis2, (int)_last_raw2);
	PX4_INFO("  Total     : %7.2f deg", (double)_last_total);
	PX4_INFO("  Direction : %6.1f deg", (double)_last_dir);
	float off1 = 0.0f, off2 = 0.0f;
	param_get(_param_axis1_off, &off1);
	param_get(_param_axis2_off, &off2);
	PX4_INFO("  offset    : axis1=%.2f  axis2=%.2f deg", (double)off1, (double)off2);
	perf_print_counter(_loop_perf);
	perf_print_counter(_err_perf);
}

void FlexSensor::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Nitto/Bend Labs 2-axis Flex Sensor Driver

### [처음 설정 순서]
  1) 센서 1개만 I2C 버스에 연결
  2) flex_sensor set_addr -X -b 3 0x14   <- 0x13->0x14로 주소 변경
  3) 전원 OFF->ON 후 i2cdetect -b 3 으로 확인
  4) 나머지 센서도 같은 방법으로 0x15, 0x16으로 변경
  5) 4개 모두 연결 후 flex_sensor status 로 동작 확인

### [영점 설정]
  flex_sensor zero   <- 현재 자세를 0도 기준으로 저장 (모든 센서 동시)
  param save         <- 재부팅 후에도 유지

### [캘리브레이션 문제 진단]
  다른 센서 대비 평평한 자세에서 raw 값이 비정상적으로 크면:
  flex_sensor factory_reset -X -b 3 -a <addr>   <- 사용자 캘리브레이션 삭제, 공장값 복원
  초기화 후에도 값이 안 돌아오면 하드웨어 손상(과도한 스트레인 등) 의심

### [실시간 데이터 확인]
  listener flex_sensor -i 0   # Sensor 1 [0x13]
  listener flex_sensor -i 1   # Sensor 2 [0x14]
  listener flex_sensor -i 2   # Sensor 3 [0x15]
  listener flex_sensor -i 3   # Sensor 4 [0x16]
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flex_sensor", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("flex_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_FLAG('2', "Force 2-axis mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("zero", "Zero all sensors (run 'param save' to persist)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("diag", "I2C bus scan + step-by-step init + 20-sample read test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_addr", "Change I2C address (connect 1 sensor only)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("factory_reset", "Clear user calibration, restore factory calibration (connect 1 sensor only)");
	PRINT_MODULE_USAGE_PARAM_FLAG('X', "External I2C bus", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 3, 1, 4, "Bus number", false);
	PRINT_MODULE_USAGE_PARAM_INT('a', 0x13, 0x08, 0x77, "Current address (auto-scan if omitted)", true);
	PRINT_MODULE_USAGE_ARG("<new addr>", "New I2C address (e.g. 0x14)", false);
}

extern "C" __EXPORT int flex_sensor_main(int argc, char *argv[])
{
	using ThisDriver = FlexSensor;
	BusCLIArguments cli{true, false};
	cli.i2c_address           = ADS_DEFAULT_ADDR;
	cli.default_i2c_frequency = ADS_BUS_CLOCK_HZ;

	int ch;

	while ((ch = cli.getOpt(argc, argv, "2")) != EOF) {
		switch (ch) {
		case '2':
			cli.custom1 = 1;  /* 2축 강제 지정 플래그 */
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DEVTYPE_UNUSED);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);

	} else if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);

	} else if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);

	} else if (!strcmp(verb, "zero")) {
		FlexSensor::request_zero();
		PX4_INFO("Zero requested - applied in ~10ms");
		PX4_INFO("Run 'param save' to persist across reboots");
		return 0;

	} else if (!strcmp(verb, "shutdown_test")) {
		FlexSensor::request_shutdown_test();
		PX4_INFO("SHUTDOWN recovery test requested - running in next RunImpl (~10ms)");
		PX4_INFO("Watch 'flex_sensor status' or syslog for results");
		return 0;

	} else if (!strcmp(verb, "diag")) {
		/* 버스만 열면 되므로 iterator 실패는 무시하고 진행 */
		iterator.next();
		I2CSPIDriverConfig config(cli, iterator, px4::wq_configurations::I2C1);
		FlexSensor *dev = new FlexSensor(config);

		if (dev->I2C::init() != PX4_OK) {
			PX4_ERR("Cannot open I2C bus %d", config.bus);
			PX4_ERR("  -> check bus number: flex_sensor diag -X -b <1..4>");
			delete dev;
			return -1;
		}

		dev->diag();
		delete dev;
		return 0;

	} else if (!strcmp(verb, "set_addr")) {
		/* 마지막 인수 = 새 I²C 주소 */
		uint8_t new_addr = (uint8_t)strtol(argv[argc - 1], nullptr, 0);

		if (new_addr < 0x08 || new_addr > 0x77) {
			PX4_ERR("Invalid address 0x%02X (valid: 0x08-0x77)", new_addr);
			PX4_ERR("Usage: flex_sensor set_addr -X -b 3 [-a <cur>] <new>");
			return -1;
		}

		if (!iterator.next()) {
			PX4_ERR("No sensor found on bus - check wiring and connect only 1 sensor");
			return -1;
		}

		I2CSPIDriverConfig config(cli, iterator, px4::wq_configurations::I2C1);
		FlexSensor *dev = new FlexSensor(config);
		int ret = dev->I2C::init();

		if (ret != PX4_OK) {
			PX4_ERR("Sensor not responding (bus %d addr 0x%02X) - check wiring",
				config.bus, config.i2c_address);
			delete dev;
			return ret;
		}

		PX4_INFO("Sensor found: bus %d addr 0x%02X", config.bus, config.i2c_address);
		ret = dev->set_device_addr(new_addr);
		px4_usleep(10000);

		if (ret == PX4_OK) {
			PX4_INFO("Address changed: 0x%02X -> 0x%02X", config.i2c_address, new_addr);
			PX4_INFO("Power cycle, then verify with 'i2cdetect -b %d'", config.bus);
		} else {
			PX4_ERR("Address change failed - check sensor connection");
		}

		delete dev;
		return ret;

	} else if (!strcmp(verb, "factory_reset")) {
		if (!iterator.next()) {
			PX4_ERR("No sensor found on bus - check wiring and connect only 1 sensor");
			return -1;
		}

		I2CSPIDriverConfig config(cli, iterator, px4::wq_configurations::I2C1);
		FlexSensor *dev = new FlexSensor(config);
		int ret = dev->I2C::init();

		if (ret != PX4_OK) {
			PX4_ERR("Sensor not responding (bus %d addr 0x%02X) - check wiring",
				config.bus, config.i2c_address);
			delete dev;
			return ret;
		}

		PX4_INFO("Sensor found: bus %d addr 0x%02X", config.bus, config.i2c_address);
		ret = dev->factory_reset();
		px4_usleep(10000);

		if (ret == PX4_OK) {
			PX4_INFO("Factory reset sent: user calibration cleared, factory calibration restored");
			PX4_INFO("Check flat-position raw value with 'flex_sensor diag' or 'flex_sensor status'");
		} else {
			PX4_ERR("Factory reset failed - check sensor connection");
		}

		delete dev;
		return ret;
	}

	ThisDriver::print_usage();
	return -1;
}
