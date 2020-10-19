/***************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file gpsfailure.h
 * Helper class for Data Link Loss Mode according to the OBC rules
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#pragma once

#include <px4_module_params.h>

#include "mission_block.h"
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>

class Navigator;

class GpsFailure : public MissionBlock, public ModuleParams
{
public:
	GpsFailure(Navigator *navigator);
	~GpsFailure() = default;

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_GPSF_LT>) _param_nav_gpsf_lt,
		(ParamFloat<px4::params::NAV_GPSF_R>) _param_nav_gpsf_r,
		(ParamFloat<px4::params::NAV_GPSF_P>) _param_nav_gpsf_p,
		(ParamFloat<px4::params::NAV_GPSF_TR>) _param_nav_gpsf_tr
	)

	enum GPSFState {
		GPSF_STATE_NONE = 0,
		GPSF_STATE_LOITER = 1,
		GPSF_STATE_TERMINATE = 2,
		GPSF_STATE_END = 3,
	} _gpsf_state{GPSF_STATE_NONE};

	hrt_abstime _timestamp_activation{0}; //*< timestamp when this mode was activated */

	orb_advert_t	_att_sp_pub{nullptr};
	uORB::Subscription<vehicle_air_data_s>	_sub_airdata;
	float _gps_failed_altitude{0};

	int	_manual_control_sub{-1};		///< notification of manual control updates */
   	int	_control_mode_sub{-1};			///< control mode subscription */

	manual_control_setpoint_s	_manual {};			///< r/c channel data */
	vehicle_control_mode_s		_control_mode {};		///< control mode */
	/**
	 * Set the GPSF item
	 */
	void		set_gpsf_item();

	/**
	 * Move to next GPSF item
	 */
	void		advance_gpsf();

	void vehicle_control_mode_poll();

	void manual_control_setpoint_poll();

};
