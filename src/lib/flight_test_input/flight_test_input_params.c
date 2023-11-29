/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file flight_test_input_params.c
 *
 * Parameters defined by the FlightTestInput lib.
 */

/**
 * Test input Enable/Disable Frequency sweep
 *
 * @boolean
 * @group Flight Test Input
 */
PARAM_DEFINE_INT32(FTI_ENABLE, 0);

/**
 * Test input Enable/Disable
 *
 * Frequency sweep test flag for estimates of the omega_min and the omega_max
 *
 * @boolean
 * @group Flight Test Input
 */
PARAM_DEFINE_INT32(FTI_EST_OMEGA, 0);

/**
 * Test Input Mode
 *
 * 0 for injecting frequency sweeps, 1 for doublet
 *
 * @min 0
 * @max 1
 * @value 0 FrequencySweep
 * @value 1 Doublet
 * @group Flight Test Input
 */
PARAM_DEFINE_INT32(FTI_MODE, 0);

/**
 * Test input excitation point
 *
 * 0 - None
 * 1 - Actuator control (MANUAL mode)
 * 2 - Rate control (RATE mode)
 * 3 - Attitude control (STABILIZED mode)
 *
 * @min 0
 * @max 3
 * @value 0 NONE
 * @value 1 ACTUATOR_CONTROL
 * @value 2 ATTITUDE_CONTROL
 * @value 3 RATE_CONTROL
 * @group Flight Test Input
 */
PARAM_DEFINE_INT32(FTI_EXCITE_POINT, 0);

/**
 * Test input excitation point
 *
 * 0 - None
 * 1 - Roll
 * 2 - Pitch
 * 3 - Yaw
 *
 * @min 0
 * @max 3
 * @value 0 NONE
 * @value 1 ROLL
 * @value 2 PITCH
 * @value 3 YAW
 * @group Flight Test Input
 */
PARAM_DEFINE_INT32(FTI_EXCITE_INDEX, 0);

/**
 * Test input bandwidth frequency at phase of the attitude response is -135deg phase
 *
 * If FTI_EST_OMEGA is disabled, this value can still be assigned to it.
 * Alternatively, this value is set to 0.
 *
 * @unit rad/s
 * @min 0.6
 * @max 4.0
 * @decimal 2
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_FS_BW_FREQ, 0.6f);

/**
 * Test input frequency at phase of the attitude response is -180deg
 *
 * If FTI_EST_OMEGA is disabled, this value can still be assigned to it.
 * Alternatively, this value is set to 0.
 *
 * @unit rad/s
 * @min 4.0
 * @max 10.0
 * @decimal 2
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_FS_180_FREQ, 5.0f);

/**
 * Test input minimum frequency sweep
 *
 * If FTI_EST_OMEGA is enabled, this value is equal to 1.0
 *
 * The value is equal to  0.5 * FTI_FS_BW_FREQ
 *
 * @unit rad/s
 * @min 0.3
 * @max 2.0
 * @decimal 2
 * @group Flight Test Input
*/
PARAM_DEFINE_FLOAT(FTI_FS_OMEGA_MIN, 0.3f);

/**
 * Test input maximum frequency sweep
 *
 * If FTI_EST_OMEGA is enabled, this value is equal to 20.0
 *
 * The value is equal to  2.5 * FTI_FS_180_FREQ
 *
 * @unit rad/s
 * @min 10.0
 * @max 25.0
 * @decimal 2
 * @group Flight Test Input
*/
PARAM_DEFINE_FLOAT(FTI_FS_OMEGA_MAX, 20.0f);

/**
 * Test input maximum time for long-period frequency sweep inputs
 *
 * The value is equal to  2*PI/ FTI_FS_OMEGA_MIN
 *
 * @unit s
 * @min 3.0
 * @max 25.0
 * @decimal 0
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_FS_T_MAX, 3.0f);

/**
 * Test input frequency sweep progression record length
 *
 * The value is equal to  5 * FTI_FS_T_MAX
 *
 * @unit s
 * @min 15.0
 * @max 125.0
 * @decimal 0
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_FS_T_REC, 15.0f);

/**
 * Test input frequency sweep amplitude
 *
 * @unit norm
 * @min 0.1
 * @max 0.2
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_FS_SWEEP_AMP, 0.1f);

/**
 * Test input stating trim duration
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 0
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_FS_T_TRIM_S, 3.0f);

/**
 * Test input ending trim duration
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 0
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_FS_T_TRIM_E, 3.0f);

/**
 * Test input fade-in duration
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 0
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_FS_T_FADE_I, 0.0f);

/**
 * Test input fade-out duration
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 0
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_FS_T_FADE_O, 0.0f);

/**
 * Test input doublet pulse length
 *
 * Length of each doublet pulse in seconds
 *
 * @unit s
 * @min 0.0
 * @max 25.0
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_PULSE_LEN, 3.0f);

/**
 * Test input doublet pulse amplitude
 *
 * Amplitude of each doublet pulse
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @group Flight Test Input
 */
PARAM_DEFINE_FLOAT(FTI_PULSE_AMP, 0.0f);
