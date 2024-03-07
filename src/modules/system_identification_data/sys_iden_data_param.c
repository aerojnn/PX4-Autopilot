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

/**
 * @file sys_iden_data_param.c
 *
 * Parameters defined by the system identification data task
 *
 * @author Nanthawat Saetun <nanthawat.jn@gmail.com>
 */


/**
 * The schedule interval in Hz
 *
 * @unit Hz
 * @min 10
 * @max 500
 * @reboot_required true
 * @group System Identification Data
 */
PARAM_DEFINE_INT32(SYSID_RATE, 80);

/**
 * Maximum aileron deflection
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group System Identification Data
 */
PARAM_DEFINE_FLOAT(SYSID_MAX_AIL_D, 1.0f);

/**
 * Maximum elevator deflection
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group System Identification Data
 */
PARAM_DEFINE_FLOAT(SYSID_MAX_ELE_D, 1.0f);

/**
 * Maximum rudder deflection
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group System Identification Data
 */
PARAM_DEFINE_FLOAT(SYSID_MAX_RUD_D, 1.0f);
