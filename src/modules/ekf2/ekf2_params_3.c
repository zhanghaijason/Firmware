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
 * EKF2/3 device ID of IMU
 * @group EKF2/3
 * @reboot_required true
 */
PARAM_DEFINE_INT32(EKF2_3_IMU_ID, 0);

/**
 * EKF/3 device ID of Magnetometer
 *
 * @group EKF2/3
 * @reboot_required true
 */
PARAM_DEFINE_INT32(EKF2_3_MAG_ID, 0);

/**
 * Magnetic declination
 *
 * @group EKF2/3
 * @volatile
 * @category system
 * @unit deg
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_3_MAG_DECL, 0);

/**
 * Learned value of magnetometer X axis bias.
 * This is the amount of X-axis magnetometer bias learned by the EKF and saved from the last flight. It must be set to zero if the ground based magnetometer calibration is repeated.
 *
 * @group EKF2/3
 * @min -0.5
 * @max 0.5
 * @reboot_required true
 * @volatile
 * @category system
 * @unit mGauss
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_3_MAGBIAS_X, 0.0f);

/**
 * Learned value of magnetometer Y axis bias.
 * This is the amount of Y-axis magnetometer bias learned by the EKF and saved from the last flight. It must be set to zero if the ground based magnetometer calibration is repeated.
 *
 * @group EKF2/3
 * @min -0.5
 * @max 0.5
 * @reboot_required true
 * @volatile
 * @category system
 * @unit mGauss
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_3_MAGBIAS_Y, 0.0f);

/**
 * Learned value of magnetometer Z axis bias.
 * This is the amount of Z-axis magnetometer bias learned by the EKF and saved from the last flight. It must be set to zero if the ground based magnetometer calibration is repeated.
 *
 * @group EKF2/3
 * @min -0.5
 * @max 0.5
 * @reboot_required true
 * @volatile
 * @category system
 * @unit mGauss
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_3_MAGBIAS_Z, 0.0f);
