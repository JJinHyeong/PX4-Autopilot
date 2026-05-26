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
 * Flex sensor 0 (AIN0) raw ADC value at flat (0 degrees)
 *
 * @group Flex Sensor
 * @min 0
 * @max 32767
 */
PARAM_DEFINE_INT32(FLEX_CAL_FLAT0, 11600);

/**
 * Flex sensor 0 (AIN0) raw ADC value at 90 degrees bend
 *
 * @group Flex Sensor
 * @min 0
 * @max 32767
 */
PARAM_DEFINE_INT32(FLEX_CAL_90D0, 9180);

/**
 * Flex sensor 1 (AIN1) raw ADC value at flat (0 degrees)
 *
 * @group Flex Sensor
 * @min 0
 * @max 32767
 */
PARAM_DEFINE_INT32(FLEX_CAL_FLAT1, 11114);

/**
 * Flex sensor 1 (AIN1) raw ADC value at 90 degrees bend
 *
 * @group Flex Sensor
 * @min 0
 * @max 32767
 */
PARAM_DEFINE_INT32(FLEX_CAL_90D1, 8550);

/**
 * Flex sensor 2 (AIN2) raw ADC value at flat (0 degrees)
 *
 * @group Flex Sensor
 * @min 0
 * @max 32767
 */
PARAM_DEFINE_INT32(FLEX_CAL_FLAT2, 10701);

/**
 * Flex sensor 2 (AIN2) raw ADC value at 90 degrees bend
 *
 * @group Flex Sensor
 * @min 0
 * @max 32767
 */
PARAM_DEFINE_INT32(FLEX_CAL_90D2, 8206);

/**
 * Flex sensor 3 (AIN3) raw ADC value at flat (0 degrees)
 *
 * @group Flex Sensor
 * @min 0
 * @max 32767
 */
PARAM_DEFINE_INT32(FLEX_CAL_FLAT3, 11354);

/**
 * Flex sensor 3 (AIN3) raw ADC value at 90 degrees bend
 *
 * @group Flex Sensor
 * @min 0
 * @max 32767
 */
PARAM_DEFINE_INT32(FLEX_CAL_90D3, 8963);
