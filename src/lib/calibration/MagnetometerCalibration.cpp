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

#include "MagnetometerCalibration.hpp"

#include "Utilities.hpp"
#include <lib/parameters/param.h>

using namespace matrix;
using namespace sensors::calibration;
using namespace time_literals;

using math::radians;

namespace sensors
{

void MagnetometerCalibration::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		ParametersUpdate();
	}
}

Vector3f MagnetometerCalibration::Correct(const Vector3f &data)
{
	return _rotation * (_scale * ((data + _power * _power_compensation) - _offset));
}

void MagnetometerCalibration::ParametersUpdate()
{
	if (_device_id == 0) {
		return;
	}

	const int calibration_index = FindCalibrationIndex("MAG", _device_id);

	if (calibration_index >= 0) {

		char str[30] {};

		if (!_external) {
			_rotation = GetBoardRotation();

		} else {
			int32_t rotation = GetCalibrationParam("MAG", "ROT", calibration_index);
			_rotation = get_rot_matrix((enum Rotation)rotation);
		}

		// CAL_MAGx_PRIO
		_priority = GetCalibrationParam("MAG", "PRIO", calibration_index);

		if (_priority < 0 || _priority > 100) {
			// reset to default
			int32_t new_priority = _external ? MAG_DEFAULT_EXTERNAL_PRIORITY : MAG_DEFAULT_PRIORITY;
			PX4_ERR("%s invalid value %d, resetting to %d", str, _priority, new_priority);
			_priority = new_priority;
			param_set_no_notification(param_find(str), &_priority);
		}

		_offset = GetCalibrationParamsVector3f("MAG", "OFF", calibration_index);
		Vector3f diag = GetCalibrationParamsVector3f("MAG", "SCALE", calibration_index);
		Vector3f offdiag = GetCalibrationParamsVector3f("MAG", "ODIAG", calibration_index);
		_power_compensation = GetCalibrationParamsVector3f("MAG", "COMP", calibration_index);

		float scale[9] {
			diag(0),    offdiag(0), offdiag(1),
			offdiag(0),    diag(1), offdiag(2),
			offdiag(1), offdiag(2),    diag(2)
		};
		_scale = Matrix3f{scale};

	} else {
		_priority = _external ? MAG_DEFAULT_EXTERNAL_PRIORITY : MAG_DEFAULT_PRIORITY;
		_offset.zero();
		_scale.setIdentity();
		_power_compensation.zero();

		_rotation.setIdentity();
	}
}

void MagnetometerCalibration::PrintStatus()
{
	PX4_INFO("%s %d EN: %d, offset: [%.4f %.4f %.4f] scale: [%.4f %.4f %.4f]", "MAG", device_id(), enabled(),
		 (double)_offset(0), (double)_offset(1), (double)_offset(2),
		 (double)_scale(0, 0), (double)_scale(1, 1), (double)_scale(2, 2));

#if defined(DEBUG_BUILD)
	_scale.print()
#endif // DEBUG_BUILD
}

} // namespace sensors
