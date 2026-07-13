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
 * @file FlexSensor.hpp
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │         Nitto / Bend Labs ADS 2-axis Flex Sensor Driver     │
 * ├─────────────────────────────────────────────────────────────┤
 * │ 센서 종류  : 차동 정전용량 방식 2축 각도 변위 센서           │
 * │ 인터페이스 : I²C (기본 주소 0x13, 버스 3)                   │
 * │ 출력       : 2축 굽힘각 (Axis1, Axis2), 단위 deg            │
 * │ 분해능     : 1/32°/LSB = 0.03125° (공식 BendLabs 기준)      │
 * │ 측정 범위  : -105° ~ +105° (양방향)                         │
 * │ 반복정밀도 : 0.18°                                           │
 * │ 샘플레이트 : 100 Hz                                          │
 * ├─────────────────────────────────────────────────────────────┤
 * │ I²C 프로토콜 (레지스터 방식 아님, 커맨드 기반)              │
 * │  - 쓰기: [CMD_BYTE, PARAM...]                               │
 * │  - 읽기: 5바이트 [PKT_TYPE | X_L X_H | Y_L Y_H]           │
 * ├─────────────────────────────────────────────────────────────┤
 * │ uORB 토픽  : flex_sensor                                    │
 * │  - axis1_deg  : Axis1 굽힘각                                │
 * │  - axis2_deg  : Axis2 굽힘각 (수직 평면)                    │
 * │  - total_deg  : 합성 굽힘각 sqrt(a1²+a2²)                  │
 * │  - direction_deg : 굽힘 방향 atan2(a2,a1)                  │
 * │  - raw_axis1  : Axis1 원시 카운트 (각도 = raw / 32.0)      │
 * │  - raw_axis2  : Axis2 원시 카운트                           │
 * │  - state      : 0=FLAT / 1=BENT / 2=TWIST                  │
 * └─────────────────────────────────────────────────────────────┘
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/flex_sensor.h>
#include <uORB/topics/debug_array.h>
#include <parameters/param.h>
#include <math.h>

/* ================================================================
 * I²C 커맨드 바이트 정의
 * 센서는 레지스터 주소 방식이 아니라 커맨드 기반 프로토콜 사용
 * 커맨드 전송: transfer(cmd_buf, len, nullptr, 0)
 * 데이터 수신: transfer(nullptr, 0, rx_buf, len)
 * ================================================================ */
#define ADS_CMD_RUN        0x00u  /* 연속 출력 모드: [0x00, 0x01]=시작, [0x00, 0x00]=정지 */
#define ADS_CMD_SPS        0x01u  /* 샘플레이트 설정: [0x01, SPS_LSB, SPS_MSB] */
#define ADS_CMD_RESET      0x02u  /* 소프트 리셋: [0x02] → 50ms 대기 필요 */
#define ADS_CMD_INT_EN     0x05u  /* DRDY 인터럽트 활성화: [0x05, 0x01]=핀 활성 (공식명 ADS_INTERRUPT_ENABLE) */
#define ADS_CMD_GET_DEV_ID 0x0Au  /* 디바이스 ID 요청: [0x0A, 0,0,0,0] → 5바이트 응답 */
#define ADS_CMD_SET_ADDR   0x04u  /* I²C 주소 변경: [0x04, new_addr, 0, 0, 0] → 센서 플래시에 영구 저장 */
#define ADS_CMD_SHUTDOWN   0x09u  /* 초저전력 대기: ~50nA, RESET으로만 복귀 가능 */

/* ================================================================
 * 응답 패킷 타입 (수신 데이터의 buf[0])
 * ================================================================ */
#define ADS_PKT_SAMPLE     0x00u  /* 각도 데이터 패킷 (정상 응답) */
#define ADS_PKT_DEV_ID     0x02u  /* 디바이스 ID 응답 패킷 */

/* ================================================================
 * 디바이스 ID (ADS_CMD_GET_DEV_ID 응답의 buf[1])
 * ================================================================ */
#define ADS_DEV_1AXIS      1u     /* 1축 센서 v1 */
#define ADS_DEV_1AXIS_V2   12u    /* 1축 센서 v2 */
#define ADS_DEV_2AXIS      2u     /* 2축 센서 v1 */
#define ADS_DEV_2AXIS_V2   22u    /* 2축 센서 v2 (Nitto) */

/* ================================================================
 * 스케일 팩터 (raw int16 → 각도 변환)
 * 공식 BendLabs 코드: sample = (float)temp / 32.0f
 * 예) raw=2880 → 2880 / 32.0 = 90.0°
 * ================================================================ */
#define ADS_SCALE_1AXIS        0.015625f  /* 1축: 1/64°/LSB (SparkFun 라이브러리 기준) */
#define ADS_SCALE_2AXIS        0.03125f   /* 2축: 1/32°/LSB (공식 BendLabs 기준: temp/32.0f) */

/* ================================================================
 * 샘플레이트 설정값 (ADS_CMD_SPS에 사용)
 * SPS 레지스터 = 16384 / target_Hz
 * 100Hz → 16384/100 = 163
 * ================================================================ */
#define ADS_SPS_100HZ      163u

/* ================================================================
 * 드라이버 기본 설정
 * ================================================================ */
#define ADS_DEFAULT_ADDR   0x13u    /* 2축 센서 기본 I²C 주소 */
#define ADS_BUS_CLOCK_HZ   400000u  /* I²C Fast Mode (400 kHz) */
#define ADS_TX_SIZE        5u       /* 2축 I²C 쓰기 프레임 크기 (공식 ADS_TRANSFER_SIZE) */
#define ADS_POLL_US        10000u   /* RunImpl period: 10ms = 100Hz (matches sensor output rate) */
#define ADS_RANGE_DEG      200.0f   /* Hard range check: rejects completely corrupted I2C reads */
#define ADS_MEDIAN_N       3        /* Median filter window size */

class FlexSensor : public device::I2C, public I2CSPIDriver<FlexSensor>
{
public:
	FlexSensor(const I2CSPIDriverConfig &config);
	~FlexSensor() override;

	static void print_usage();

	int  init() override;
	void RunImpl();
	void print_status() override;
	int  set_device_addr(uint8_t new_addr);

	/* 영점 요청: flex_sensor zero 명령에서 호출 */
	static void request_zero() { _zero_mask = 0x0F; }

	/* SHUTDOWN 복구 테스트 요청: flex_sensor shutdown_test 명령에서 호출 */
	static void request_shutdown_test() { _do_shutdown_test = true; }

	/* I2C 버스 진단: 주소 스캔 + 단계별 init + 연속 샘플 출력 */
	void diag();

private:
	void run_shutdown_test();
	/* I²C 커맨드 전송 */
	int send_cmd(const uint8_t *buf, uint8_t len);

	/* I²C 데이터 수신 */
	int read_data(uint8_t *buf, uint8_t len);

	/* 디바이스 ID 읽기 → 1축/2축 자동 판별 */
	int get_device_id(uint8_t &dev_id);

	/* uORB 발행 (멀티 인스턴스: 센서마다 별도 인스턴스 자동 할당) */
	uORB::PublicationMulti<flex_sensor_s> _pub{ORB_ID(flex_sensor)};
	uORB::PublicationMulti<debug_array_s> _debug_pub{ORB_ID(debug_array)};

	/* 성능 카운터 */
	perf_counter_t _loop_perf;  /* RunImpl 실행 시간 */
	perf_counter_t _err_perf;   /* I²C 에러 횟수 */

	/* 센서 상태 */
	uint8_t _dev_id{0};          /* GET_DEV_ID 응답값 */
	bool    _is_2axis{false};    /* true = 2축 모드 */
	float   _scale{ADS_SCALE_2AXIS}; /* 스케일 팩터 (°/LSB) */

	/* 센서 인덱스 (주소 기반: 0x13→0, 0x14→1, 0x15→2, 0x16→3) */
	int _sensor_idx{0};

	/* 센서별 영점 오프셋 파라미터 핸들 */
	param_t _param_axis1_off{PARAM_INVALID};
	param_t _param_axis2_off{PARAM_INVALID};

	/* 마지막 측정값 (status 출력용) */
	int16_t _last_raw1{0};
	int16_t _last_raw2{0};
	float   _last_axis1{0.0f};
	float   _last_axis2{0.0f};
	float   _last_total{0.0f};
	float   _last_dir{0.0f};
	uint32_t _read_count{0};

	/* Median filter circular buffers */
	float   _hist1[ADS_MEDIAN_N]{};
	float   _hist2[ADS_MEDIAN_N]{};
	uint8_t _hist_idx{0};

	/* 영점 요청 마스크 (비트 0~3 = 센서 0~3)
	 * flex_sensor zero 명령 시 해당 비트 세트 → 다음 RunImpl에서 처리 */
	static volatile uint8_t _zero_mask;

	/* SHUTDOWN 복구 테스트 플래그: RunImpl에서 once 실행 후 clear */
	static volatile bool _do_shutdown_test;
};
