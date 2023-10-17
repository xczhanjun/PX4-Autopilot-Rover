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

#include "ekf.h"

#include "aux_global_position.hpp"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

void AuxGlobalPosition::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{

#if defined(MODULE_NAME)

	if (_aux_global_position_sub.updated()) {

		aux_global_position_s aux_global_position{};
		_aux_global_position_sub.copy(&aux_global_position);

		const int64_t time_us = aux_global_position.timestamp_sample - static_cast<int64_t>(_param_ekf2_agp_delay.get() * 1000);

		AuxGlobalPositionSample sample{};
		sample.time_us = time_us;
		sample.latitude = aux_global_position.latitude;
		sample.longitude = aux_global_position.longitude;
		sample.positional_uncertainty = aux_global_position.positional_uncertainty;
		sample.position_valid = aux_global_position.valid;
		sample.altitude = aux_global_position.altitude_agl;
		sample.heading = aux_global_position.heading;
		sample.heading_valid = aux_global_position.heading_valid;

		_aux_global_position_buffer.push(sample);

		_time_last_buffer_push = imu_delayed.time_us;
	}

#endif // MODULE_NAME

	AuxGlobalPositionSample sample;

	if (_aux_global_position_buffer.pop_first_older_than(imu_delayed.time_us, &sample)) {

		if (!(_param_ekf2_agp_ctrl.get() & static_cast<int32_t>(Ctrl::HPOS))) {
			return;
		}

		estimator_aid_source2d_s aid_src{};
		Vector2f position;

		if (ekf.global_origin_valid()) {
			position = ekf.global_origin().project(sample.latitude, sample.longitude);
			//const float hgt = ekf.getEkfGlobalOriginAltitude() - (float)sample.altitude;
			// relax the upper observation noise limit which prevents bad measurements perturbing the position estimate
			float pos_noise = math::max(sample.positional_uncertainty, _param_ekf2_agp_noise.get());
			const float pos_var = sq(pos_noise);
			const Vector2f pos_obs_var(pos_var, pos_var);
			ekf.updateHorizontalPositionAidSrcStatus(sample.time_us,
					position,                                   // observation
					pos_obs_var,                                // observation variance
					math::max(_param_ekf2_agp_gate.get(), 1.f), // innovation gate
					aid_src);
		}

		const bool starting_conditions = sample.position_valid
						   && ekf.control_status_flags().yaw_align;
		const bool continuing_conditions = starting_conditions
						   && ekf.global_origin_valid();

		switch (_state) {
		case State::stopped:
			if (starting_conditions) {
				_state = State::starting;
			}
			break;

		case State::active:
			if (continuing_conditions) {
				ekf.fuseHorizontalPosition(aid_src);

			} else {
				_state = State::stopping;
			}
			break;

		default:
			break;
		}

		switch (_state) {
		case State::starting:
			if (!ekf.global_origin_valid()) {
				// Try to initialize using measurement
				if (ekf.setEkfGlobalOrigin(sample.latitude, sample.longitude, sample.altitude, sample.positional_uncertainty)) {
					ekf.enableControlStatusAuxGpos();
					_state = State::active;

				} else {
					_state = State::stopping;
				}
			}
			break;

		case State::stopping:
			ekf.disableControlStatusAuxGpos();
			_state = State::stopped;
			break;

		case State::resetting:
			_state = State::active;
			break;

		default:
			break;
		}

#if defined(MODULE_NAME)
		aid_src.timestamp = hrt_absolute_time();
		_estimator_aid_src_aux_global_position_pub.publish(aid_src);
#endif // MODULE_NAME
	} else if ((_state != State::stopped) && isTimedOut(_time_last_buffer_push, imu_delayed.time_us, (uint64_t)5e6)) {
		ekf.disableControlStatusAuxGpos();
		_state = State::stopped;
		ECL_WARN("Aux global position data stopped");
	}
}

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
