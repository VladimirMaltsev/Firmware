/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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

#include <uORB/topics/actuator_controls.h>
#include <modules/px4iofirmware/protocol.h>
#include <modules/px4iofirmware/px4io.h>
#include <uORB/topics/landing_gear.h>
#include <drivers/drv_tone_alarm.h>
#include "FixedwingPositionControl.hpp"

extern "C" __EXPORT int fw_pos_control_l1_main(int argc, char *argv[]);


FixedwingPositionControl::FixedwingPositionControl() :
        ModuleParams(nullptr),
        _sub_airspeed(ORB_ID(airspeed)),
        _sub_sensors(ORB_ID(sensor_bias)),
        _loop_perf(perf_alloc(PC_ELAPSED, "fw_l1_control")),
        _launchDetector(this),
        _runway_takeoff(this) {

    _parameter_handles.l1_period = param_find("FW_L1_PERIOD");
    _parameter_handles.l1_damping = param_find("FW_L1_DAMPING");
    _parameter_handles.roll_slew_deg_sec = param_find("FW_L1_R_SLEW_MAX");

    _parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
    _parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
    _parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");
    _parameter_handles.airspeed_disabled = param_find("FW_ARSP_MODE");

    _parameter_handles.pitch_limit_min = param_find("FW_P_LIM_MIN");
    _parameter_handles.pitch_limit_max = param_find("FW_P_LIM_MAX");
    _parameter_handles.roll_limit = param_find("FW_R_LIM");
    _parameter_handles.throttle_min = param_find("FW_THR_MIN");
    _parameter_handles.throttle_max = param_find("FW_THR_MAX");
    _parameter_handles.throttle_idle = param_find("FW_THR_IDLE");
    _parameter_handles.throttle_slew_max = param_find("FW_THR_SLEW_MAX");
    _parameter_handles.throttle_cruise = param_find("FW_THR_CRUISE");
    _parameter_handles.throttle_alt_scale = param_find("FW_THR_ALT_SCL");
    _parameter_handles.throttle_land_max = param_find("FW_THR_LND_MAX");
    _parameter_handles.man_roll_max_deg = param_find("FW_MAN_R_MAX");
    _parameter_handles.man_pitch_max_deg = param_find("FW_MAN_P_MAX");
    _parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
    _parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

    _parameter_handles.land_slope_angle = param_find("FW_LND_ANG");
    _parameter_handles.land_H1_virt = param_find("FW_LND_HVIRT");
    _parameter_handles.land_flare_alt_relative = param_find("FW_LND_FLALT");
    _parameter_handles.land_flare_pitch_min_deg = param_find("FW_LND_FL_PMIN");
    _parameter_handles.land_flare_pitch_max_deg = param_find("FW_LND_FL_PMAX");
    _parameter_handles.land_thrust_lim_alt_relative = param_find("FW_LND_TLALT");
    _parameter_handles.land_heading_hold_horizontal_distance = param_find("FW_LND_HHDIST");
    _parameter_handles.land_use_terrain_estimate = param_find("FW_LND_USETER");
    _parameter_handles.land_early_config_change = param_find("FW_LND_EARLYCFG");
    _parameter_handles.land_airspeed_scale = param_find("FW_LND_AIRSPD_SC");
    _parameter_handles.land_throtTC_scale = param_find("FW_LND_THRTC_SC");

    _parameter_handles.loiter_radius = param_find("NAV_LOITER_RAD");
    _parameter_handles.sys_autostart = param_find("SYS_AUTOSTART");

    _parameter_handles.time_const = param_find("FW_T_TIME_CONST");
    _parameter_handles.time_const_throt = param_find("FW_T_THRO_CONST");
    _parameter_handles.min_sink_rate = param_find("FW_T_SINK_MIN");
    _parameter_handles.max_sink_rate = param_find("FW_T_SINK_MAX");
    _parameter_handles.max_climb_rate = param_find("FW_T_CLMB_MAX");
    _parameter_handles.climbout_diff = param_find("FW_CLMBOUT_DIFF");
    _parameter_handles.throttle_damp = param_find("FW_T_THR_DAMP");
    _parameter_handles.integrator_gain = param_find("FW_T_INTEG_GAIN");
    _parameter_handles.vertical_accel_limit = param_find("FW_T_VERT_ACC");
    _parameter_handles.height_comp_filter_omega = param_find("FW_T_HGT_OMEGA");
    _parameter_handles.speed_comp_filter_omega = param_find("FW_T_SPD_OMEGA");
    _parameter_handles.roll_throttle_compensation = param_find("FW_T_RLL2THR");
    _parameter_handles.speed_weight = param_find("FW_T_SPDWEIGHT");
    _parameter_handles.pitch_damping = param_find("FW_T_PTCH_DAMP");
    _parameter_handles.heightrate_p = param_find("FW_T_HRATE_P");
    _parameter_handles.heightrate_ff = param_find("FW_T_HRATE_FF");
    _parameter_handles.speedrate_p = param_find("FW_T_SRATE_P");
    _parameter_handles.pwm_parachute_release = param_find("PWM_PRCHT_REL");
    _parameter_handles.pwm_parachute_drop = param_find("PWM_PRCHT_DROP");

    // if vehicle is vtol these handles will be set when we get the vehicle status
    _parameter_handles.airspeed_trans = PARAM_INVALID;
    _parameter_handles.vtol_type = PARAM_INVALID;

    // initialize to invalid vtol type
    _parameters.vtol_type = -1;

    /* fetch initial parameter values */
    parameters_update();
}

FixedwingPositionControl::~FixedwingPositionControl() {
    perf_free(_loop_perf);
}

int
FixedwingPositionControl::parameters_update() {
    updateParams();

    param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
    param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
    param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));
    param_get(_parameter_handles.airspeed_disabled, &(_parameters.airspeed_disabled));
    param_get(_parameter_handles.pitch_limit_min, &(_parameters.pitch_limit_min));
    param_get(_parameter_handles.pitch_limit_max, &(_parameters.pitch_limit_max));
    param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
    param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
    param_get(_parameter_handles.throttle_idle, &(_parameters.throttle_idle));
    param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));
    param_get(_parameter_handles.throttle_alt_scale, &(_parameters.throttle_alt_scale));
    param_get(_parameter_handles.throttle_land_max, &(_parameters.throttle_land_max));

    param_get(_parameter_handles.man_roll_max_deg, &_parameters.man_roll_max_rad);
    _parameters.man_roll_max_rad = radians(_parameters.man_roll_max_rad);
    param_get(_parameter_handles.man_pitch_max_deg, &_parameters.man_pitch_max_rad);
    _parameters.man_pitch_max_rad = radians(_parameters.man_pitch_max_rad);

    param_get(_parameter_handles.rollsp_offset_deg, &_parameters.rollsp_offset_rad);
    _parameters.rollsp_offset_rad = radians(_parameters.rollsp_offset_rad);
    param_get(_parameter_handles.pitchsp_offset_deg, &_parameters.pitchsp_offset_rad);
    _parameters.pitchsp_offset_rad = radians(_parameters.pitchsp_offset_rad);

    param_get(_parameter_handles.climbout_diff, &(_parameters.climbout_diff));

    param_get(_parameter_handles.land_heading_hold_horizontal_distance,
              &(_parameters.land_heading_hold_horizontal_distance));
    param_get(_parameter_handles.land_flare_pitch_min_deg, &(_parameters.land_flare_pitch_min_deg));
    param_get(_parameter_handles.land_flare_pitch_max_deg, &(_parameters.land_flare_pitch_max_deg));
    param_get(_parameter_handles.land_use_terrain_estimate, &(_parameters.land_use_terrain_estimate));
    param_get(_parameter_handles.land_early_config_change, &(_parameters.land_early_config_change));
    param_get(_parameter_handles.land_airspeed_scale, &(_parameters.land_airspeed_scale));
    param_get(_parameter_handles.land_throtTC_scale, &(_parameters.land_throtTC_scale));
    param_get(_parameter_handles.loiter_radius, &(_parameters.loiter_radius));
    param_get(_parameter_handles.sys_autostart, &(_parameters.sys_autostart));

    // VTOL parameter VTOL_TYPE
    if (_parameter_handles.vtol_type != PARAM_INVALID) {
        param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);
    }

    // VTOL parameter VT_ARSP_TRANS
    if (_parameter_handles.airspeed_trans != PARAM_INVALID) {
        param_get(_parameter_handles.airspeed_trans, &_parameters.airspeed_trans);
    }


    float v = 0.0f;


    // L1 control parameters

    if (param_get(_parameter_handles.l1_damping, &v) == PX4_OK) {
        _l1_control.set_l1_damping(v);
    }

    if (param_get(_parameter_handles.l1_period, &v) == PX4_OK) {
        _l1_control.set_l1_period(v);
    }

    if (param_get(_parameter_handles.roll_limit, &v) == PX4_OK) {
        _l1_control.set_l1_roll_limit(radians(v));
    }

    if (param_get(_parameter_handles.roll_slew_deg_sec, &v) == PX4_OK) {
        _l1_control.set_roll_slew_rate(radians(v));
    }

    // TECS parameters

    param_get(_parameter_handles.max_climb_rate, &(_parameters.max_climb_rate));
    _tecs.set_max_climb_rate(_parameters.max_climb_rate);

    param_get(_parameter_handles.max_sink_rate, &(_parameters.max_sink_rate));
    _tecs.set_max_sink_rate(_parameters.max_sink_rate);

    param_get(_parameter_handles.speed_weight, &(_parameters.speed_weight));
    _tecs.set_speed_weight(_parameters.speed_weight);

    _tecs.set_indicated_airspeed_min(_parameters.airspeed_min);
    _tecs.set_indicated_airspeed_max(_parameters.airspeed_max);

    if (param_get(_parameter_handles.time_const_throt, &(_parameters.time_const_throt)) == PX4_OK) {
        _tecs.set_time_const_throt(_parameters.time_const_throt);
    }

    if (param_get(_parameter_handles.time_const, &v) == PX4_OK) {
        _tecs.set_time_const(v);
    }

    if (param_get(_parameter_handles.min_sink_rate, &v) == PX4_OK) {
        _tecs.set_min_sink_rate(v);
    }

    if (param_get(_parameter_handles.throttle_damp, &v) == PX4_OK) {
        _tecs.set_throttle_damp(v);
    }

    if (param_get(_parameter_handles.integrator_gain, &v) == PX4_OK) {
        _tecs.set_integrator_gain(v);
    }

    if (param_get(_parameter_handles.throttle_slew_max, &v) == PX4_OK) {
        _tecs.set_throttle_slewrate(v);
    }

    if (param_get(_parameter_handles.vertical_accel_limit, &v) == PX4_OK) {
        _tecs.set_vertical_accel_limit(v);
    }

    if (param_get(_parameter_handles.height_comp_filter_omega, &v) == PX4_OK) {
        _tecs.set_height_comp_filter_omega(v);
    }

    if (param_get(_parameter_handles.speed_comp_filter_omega, &v) == PX4_OK) {
        _tecs.set_speed_comp_filter_omega(v);
    }

    if (param_get(_parameter_handles.roll_throttle_compensation, &v) == PX4_OK) {
        _tecs.set_roll_throttle_compensation(v);
    }

    if (param_get(_parameter_handles.pitch_damping, &v) == PX4_OK) {
        _tecs.set_pitch_damping(v);
    }

    if (param_get(_parameter_handles.heightrate_p, &v) == PX4_OK) {
        _tecs.set_heightrate_p(v);
    }

    if (param_get(_parameter_handles.heightrate_ff, &v) == PX4_OK) {
        _tecs.set_heightrate_ff(v);
    }

    if (param_get(_parameter_handles.speedrate_p, &v) == PX4_OK) {
        _tecs.set_speedrate_p(v);
    }


    // Landing slope

    float land_slope_angle = 0.0f;
    param_get(_parameter_handles.land_slope_angle, &land_slope_angle);

    float land_flare_alt_relative = 0.0f;
    param_get(_parameter_handles.land_flare_alt_relative, &land_flare_alt_relative);

    float land_thrust_lim_alt_relative = 0.0f;
    param_get(_parameter_handles.land_thrust_lim_alt_relative, &land_thrust_lim_alt_relative);

    float land_H1_virt = 0.0f;
    param_get(_parameter_handles.land_H1_virt, &land_H1_virt);

    /* check if negative value for 2/3 of flare altitude is set for throttle cut */
    if (land_thrust_lim_alt_relative < 0.0f) {
        land_thrust_lim_alt_relative = 0.66f * land_flare_alt_relative;
    }

    _landingslope.update(radians(land_slope_angle), land_flare_alt_relative, land_thrust_lim_alt_relative,
                         land_H1_virt);

    landing_status_publish();

    // sanity check parameters
    if ((_parameters.airspeed_max < _parameters.airspeed_min) ||
        (_parameters.airspeed_max < 5.0f) ||
        (_parameters.airspeed_min > 100.0f) ||
        (_parameters.airspeed_trim < _parameters.airspeed_min) ||
        (_parameters.airspeed_trim > _parameters.airspeed_max)) {

        mavlink_log_critical(&_mavlink_log_pub, "Airspeed parameters invalid");

        return PX4_ERROR;
    }

    return PX4_OK;
}

void
FixedwingPositionControl::vehicle_control_mode_poll() {
    bool updated = false;

    orb_check(_control_mode_sub, &updated);

    if (updated) {
        const bool was_armed = _control_mode.flag_armed;

        if (orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode) == PX4_OK) {

            // reset state when arming
            if (!was_armed && _control_mode.flag_armed) {
                reset_takeoff_state(true);
                reset_landing_state();
            }
        }
    }
}

void
FixedwingPositionControl::vehicle_command_poll() {
    bool updated;

    orb_check(_vehicle_command_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &_vehicle_command);
        handle_command();
    }
}

void
FixedwingPositionControl::vehicle_status_poll() {
    bool updated;

    orb_check(_vehicle_status_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

        /* set correct uORB ID, depending on if vehicle is VTOL or not */
        if (_attitude_setpoint_id == nullptr) {
            if (_vehicle_status.is_vtol) {
                _attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

                _parameter_handles.airspeed_trans = param_find("VT_ARSP_TRANS");
                _parameter_handles.vtol_type = param_find("VT_TYPE");

                parameters_update();

            } else {
                _attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
            }
        }
    }
}

void
FixedwingPositionControl::vehicle_land_detected_poll() {
    bool updated;

    orb_check(_vehicle_land_detected_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
    }
}

void
FixedwingPositionControl::manual_control_setpoint_poll() {
    bool manual_updated;

    /* Check if manual setpoint has changed */
    orb_check(_manual_control_sub, &manual_updated);

    if (manual_updated) {
       // mavlink_log_critical(&_mavlink_log_pub, "Updated manual control");
        _manual_mode_last_updated = hrt_absolute_time();
        orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
    }
}

void
FixedwingPositionControl::airspeed_poll() {
    bool airspeed_valid = _airspeed_valid;

    if (!_parameters.airspeed_disabled && _sub_airspeed.update()) {

        const airspeed_s &as = _sub_airspeed.get();

        if (PX4_ISFINITE(as.indicated_airspeed_m_s)
            && PX4_ISFINITE(as.true_airspeed_m_s)
            && (as.indicated_airspeed_m_s > 0.0f)
            && !_vehicle_status.aspd_use_inhibit) {

            _airspeed_last_valid = as.timestamp;
            _airspeed = as.indicated_airspeed_m_s;

            _eas2tas = constrain(as.true_airspeed_m_s / as.indicated_airspeed_m_s, 0.9f, 2.0f);

            if (climbout_completed && (hrt_elapsed_time(&_airspeed_last_valid) > 1_s) && (_airspeed > (_parameters.airspeed_max + 1.f) || _airspeed < (_parameters.airspeed_min - 1.f))){
                airspeed_valid = false;
            }else {
                airspeed_valid = true;
            }
        }

    } else {
        // no airspeed updates for one second
        if (airspeed_valid && (hrt_elapsed_time(&_airspeed_last_valid) > 1_s)) {
            airspeed_valid = false;
        }
    }

    // update TECS if validity changed
    if (airspeed_valid != _airspeed_valid) {
        _tecs.enable_airspeed(airspeed_valid);
        _airspeed_valid = airspeed_valid;
    }
}

void
FixedwingPositionControl::vehicle_attitude_poll() {
    /* check if there is a new position */
    bool updated;
    orb_check(_vehicle_attitude_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_att);
    }

    /* set rotation matrix and euler angles */
    _R_nb = Quatf(_att.q);

    // if the vehicle is a tailsitter we have to rotate the attitude by the pitch offset
    // between multirotor and fixed wing flight
    if (_parameters.vtol_type == vtol_type::TAILSITTER && _vehicle_status.is_vtol) {
        Dcmf R_offset = Eulerf(0, M_PI_2_F, 0);
        _R_nb = _R_nb * R_offset;
    }

    Eulerf euler_angles(_R_nb);
    _roll = euler_angles(0);
    _pitch = euler_angles(1);
    _yaw = euler_angles(2);
}

void
FixedwingPositionControl::position_setpoint_triplet_poll() {
    /* check if there is a new setpoint */
    bool pos_sp_triplet_updated;
    orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

    if (pos_sp_triplet_updated) {
        orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
    }
}

float
FixedwingPositionControl::get_demanded_airspeed() {
    float altctrl_airspeed = constrain(_manual.z, _parameters.airspeed_min, _parameters.airspeed_max);
    return altctrl_airspeed;
}

float
FixedwingPositionControl::calculate_target_airspeed(float airspeed_demand) {
    /*
     * Calculate accelerated stall airspeed factor from commanded bank angle and use it to increase minimum airspeed.
     *
     *  We don't know the stall speed of the aircraft, but assuming user defined
     *  minimum airspeed (FW_AIRSPD_MIN) is slightly larger than stall speed
     *  this is close enough.
     *
     * increase lift vector to balance additional weight in bank
     *  cos(bank angle) = W/L = 1/n
     *   n is the load factor
     *
     * lift is proportional to airspeed^2 so the increase in stall speed is
     *  Vsacc = Vs * sqrt(n)
     *
     */
    float adjusted_min_airspeed = _parameters.airspeed_min;

    if (_airspeed_valid && PX4_ISFINITE(_att_sp.roll_body)) {

        adjusted_min_airspeed = constrain(_parameters.airspeed_min / sqrtf(cosf(_att_sp.roll_body)),
                                          _parameters.airspeed_min,
                                          _parameters.airspeed_max);
    }

    // add minimum ground speed undershoot (only non-zero in presence of sufficient wind)
    // sanity check: limit to range
    float airspd_max = _parameters.airspeed_max;
    if (!_airspeed_valid){
        airspd_max = _parameters.airspeed_trim;
    }
    return constrain(airspeed_demand + _groundspeed_undershoot, adjusted_min_airspeed, airspd_max);
}

void
FixedwingPositionControl::calculate_gndspeed_undershoot(const Vector2f &curr_pos,
                                                        const Vector2f &ground_speed,
                                                        const position_setpoint_s &pos_sp_prev,
                                                        const position_setpoint_s &pos_sp_curr) {
    if (pos_sp_curr.valid && !_l1_control.circle_mode()) {
        /* rotate ground speed vector with current attitude */
        Vector2f yaw_vector(_R_nb(0, 0), _R_nb(1, 0));
        yaw_vector.normalize();
        float ground_speed_body = yaw_vector * ground_speed;

        /* The minimum desired ground speed is the minimum airspeed projected on to the ground using the altitude and horizontal difference between the waypoints if available*/
        float distance = 0.0f;
        float delta_altitude = 0.0f;

        if (pos_sp_prev.valid) {
            distance = get_distance_to_next_waypoint(pos_sp_prev.lat, pos_sp_prev.lon, pos_sp_curr.lat,
                                                     pos_sp_curr.lon);
            delta_altitude = pos_sp_curr.alt - pos_sp_prev.alt;

        } else {
            distance = get_distance_to_next_waypoint((double) curr_pos(0), (double) curr_pos(1), pos_sp_curr.lat,
                                                     pos_sp_curr.lon);
            delta_altitude = pos_sp_curr.alt - _global_pos.alt;
        }

        float ground_speed_desired = _parameters.airspeed_min * cosf(atan2f(delta_altitude, distance));

        /*
         * Ground speed undershoot is the amount of ground velocity not reached
         * by the plane. Consequently it is zero if airspeed is >= min ground speed
         * and positive if airspeed < min ground speed.
         *
         * This error value ensures that a plane (as long as its throttle capability is
         * not exceeded) travels towards a waypoint (and is not pushed more and more away
         * by wind). Not countering this would lead to a fly-away.
         */
        _groundspeed_undershoot = max(ground_speed_desired - ground_speed_body, 0.0f);

    } else {
        _groundspeed_undershoot = 0.0f;
    }
}

void
FixedwingPositionControl::tecs_status_publish() {
    tecs_status_s t = {};

    switch (_tecs.tecs_mode()) {
        case TECS::ECL_TECS_MODE_NORMAL:
            t.mode = tecs_status_s::TECS_MODE_NORMAL;
            break;

        case TECS::ECL_TECS_MODE_UNDERSPEED:
            t.mode = tecs_status_s::TECS_MODE_UNDERSPEED;
            break;

        case TECS::ECL_TECS_MODE_BAD_DESCENT:
            t.mode = tecs_status_s::TECS_MODE_BAD_DESCENT;
            break;

        case TECS::ECL_TECS_MODE_CLIMBOUT:
            t.mode = tecs_status_s::TECS_MODE_CLIMBOUT;
            break;
    }

    t.altitude_sp = _tecs.hgt_setpoint_adj();
    t.altitude_filtered = _tecs.vert_pos_state();

    t.airspeed_sp = _tecs.TAS_setpoint_adj();
    t.airspeed_filtered = _tecs.tas_state();

    t.height_rate_setpoint = _tecs.hgt_rate_setpoint();
    t.height_rate = _tecs.vert_vel_state();

    t.airspeed_derivative_sp = _tecs.TAS_rate_setpoint();
    t.airspeed_derivative = _tecs.speed_derivative();

    t.total_energy_error = _tecs.STE_error();
    t.total_energy_rate_error = _tecs.STE_rate_error();

    t.energy_distribution_error = _tecs.SEB_error();
    t.energy_distribution_rate_error = _tecs.SEB_rate_error();

    t.throttle_integ = _tecs.throttle_integ_state();
    t.pitch_integ = _tecs.pitch_integ_state();

    t.timestamp = hrt_absolute_time();

    if (_tecs_status_pub != nullptr) {
        orb_publish(ORB_ID(tecs_status), _tecs_status_pub, &t);

    } else {
        _tecs_status_pub = orb_advertise(ORB_ID(tecs_status), &t);
    }
}

void
FixedwingPositionControl::status_publish() {
    position_controller_status_s pos_ctrl_status = {};

    pos_ctrl_status.nav_roll = _att_sp.roll_body;
    pos_ctrl_status.nav_pitch = _att_sp.pitch_body;
    pos_ctrl_status.nav_bearing = _l1_control.nav_bearing();

    pos_ctrl_status.target_bearing = _l1_control.target_bearing();
    pos_ctrl_status.xtrack_error = _l1_control.crosstrack_error();

    pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
                                                            _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

    pos_ctrl_status.acceptance_radius = _l1_control.switch_distance(500.0f);

    pos_ctrl_status.yaw_acceptance = NAN;

    pos_ctrl_status.timestamp = hrt_absolute_time();

    if (_pos_ctrl_status_pub != nullptr) {
        orb_publish(ORB_ID(position_controller_status), _pos_ctrl_status_pub, &pos_ctrl_status);

    } else {
        _pos_ctrl_status_pub = orb_advertise(ORB_ID(position_controller_status), &pos_ctrl_status);
    }
}

void
FixedwingPositionControl::landing_status_publish() {
    position_controller_landing_status_s pos_ctrl_landing_status = {};

    pos_ctrl_landing_status.slope_angle_rad = _landingslope.landing_slope_angle_rad();
    pos_ctrl_landing_status.horizontal_slope_displacement = _landingslope.horizontal_slope_displacement();
    pos_ctrl_landing_status.flare_length = _landingslope.flare_length();

    pos_ctrl_landing_status.abort_landing = _land_abort;

    pos_ctrl_landing_status.timestamp = hrt_absolute_time();

    if (_pos_ctrl_landing_status_pub != nullptr) {
        orb_publish(ORB_ID(position_controller_landing_status), _pos_ctrl_landing_status_pub, &pos_ctrl_landing_status);

    } else {
        _pos_ctrl_landing_status_pub = orb_advertise(ORB_ID(position_controller_landing_status),
                                                     &pos_ctrl_landing_status);
    }
}

void
FixedwingPositionControl::abort_landing(bool abort) {
    // only announce changes
    if (abort && !_land_abort) {
        mavlink_log_critical(&_mavlink_log_pub, "Landing aborted");
    }

    _land_abort = abort;
    landing_status_publish();
}

void
FixedwingPositionControl::get_waypoint_heading_distance(float heading, position_setpoint_s &waypoint_prev,
                                                        position_setpoint_s &waypoint_next, bool flag_init) {
    position_setpoint_s temp_prev = waypoint_prev;
    position_setpoint_s temp_next = waypoint_next;

    if (flag_init) {
        // previous waypoint: HDG_HOLD_SET_BACK_DIST meters behind us
        waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading + radians(180.0f),
                                           HDG_HOLD_SET_BACK_DIST, &temp_prev.lat, &temp_prev.lon);

        // next waypoint: HDG_HOLD_DIST_NEXT meters in front of us
        waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading,
                                           HDG_HOLD_DIST_NEXT, &temp_next.lat, &temp_next.lon);

    } else {
        // use the existing flight path from prev to next

        // previous waypoint: shifted HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST
        create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
                                           HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST, &temp_prev.lat,
                                           &temp_prev.lon);

        // next waypoint: shifted -(HDG_HOLD_DIST_NEXT + HDG_HOLD_REACHED_DIST)
        create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
                                           -(HDG_HOLD_REACHED_DIST + HDG_HOLD_DIST_NEXT), &temp_next.lat,
                                           &temp_next.lon);
    }

    waypoint_prev = temp_prev;
    waypoint_prev.alt = _hold_alt;
    waypoint_prev.valid = true;

    waypoint_next = temp_next;
    waypoint_next.alt = _hold_alt;
    waypoint_next.valid = true;
}

float
FixedwingPositionControl::get_terrain_altitude_takeoff(float takeoff_alt,
                                                       const vehicle_global_position_s &global_pos) {
    if (PX4_ISFINITE(global_pos.terrain_alt) && global_pos.terrain_alt_valid) {
        return global_pos.terrain_alt;
    }

    return takeoff_alt;
}

bool
FixedwingPositionControl::update_desired_altitude(float dt) {
    /*
     * if the desired altitude less then 100m keep current altitude
     */
    const float deadBand = 100.0f;

    /* Climbout mode sets maximum throttle and pitch up */
    bool climbout_mode = false;

    /*
     * Reset the hold altitude to the current altitude if the uncertainty
     * changes significantly.
     * This is to guard against uncommanded altitude changes
     * when the altitude certainty increases or decreases.
     */

    if (fabsf(_althold_epv - _global_pos.epv) > ALTHOLD_EPV_RESET_THRESH) {
        _hold_alt = _global_pos.alt;
        _althold_epv = _global_pos.epv;
    }

    /*
     * Manual control has as convention the rotation around
     * an axis. Positive X means to rotate positively around
     * the X axis in NED frame, which is pitching down
     */
    if (_manual.x > deadBand) {
        _hold_alt = _manual.x;
    } else {
        _hold_alt = _global_pos.alt;
        _althold_epv = _global_pos.epv;
    }

    if (_vehicle_status.is_vtol) {
        if (_vehicle_status.is_rotary_wing || _vehicle_status.in_transition_mode) {
            _hold_alt = _global_pos.alt;
        }
    }

    return climbout_mode;
}

bool
FixedwingPositionControl::in_takeoff_situation() {
    // a VTOL does not need special takeoff handling
    if (_vehicle_status.is_vtol) {
        return false;
    }

    // in air for < 10s
    const hrt_abstime delta_takeoff = 10_s;

    return (hrt_elapsed_time(&_time_went_in_air) < delta_takeoff)
           && (_global_pos.alt <= _takeoff_ground_alt + _parameters.climbout_diff);
}

void
FixedwingPositionControl::do_takeoff_help(float *hold_altitude, float *pitch_limit_min) {
    /* demand "climbout_diff" m above ground if user switched into this mode during takeoff */
    if (in_takeoff_situation()) {
        *hold_altitude = _takeoff_ground_alt + _parameters.climbout_diff;
        *pitch_limit_min = radians(10.0f);
    }
}

bool
FixedwingPositionControl::control_position(const Vector2f &curr_pos, const Vector2f &ground_speed,
                                           const position_setpoint_s &pos_sp_prev,
                                           const position_setpoint_s &pos_sp_curr,
                                           const position_setpoint_s &pos_sp_next) {
    float dt = 0.01f;

    if (_control_position_last_called > 0) {
        dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
    }

    _control_position_last_called = hrt_absolute_time();

    _l1_control.set_dt(dt);

    /* only run position controller in fixed-wing mode and during transitions for VTOL */
    if (_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode) {
        _control_mode_current = FW_POSCTRL_MODE_OTHER;
        return false;
    }

    bool setpoint = true;

    _att_sp.fw_control_yaw = false;        // by default we don't want yaw to be contoller directly with rudder
    _att_sp.apply_flaps = vehicle_attitude_setpoint_s::FLAPS_OFF;        // by default we don't use flaps

    calculate_gndspeed_undershoot(curr_pos, ground_speed, pos_sp_prev, pos_sp_curr);

    Vector2f nav_speed_2d{ground_speed};

    if (_airspeed_valid) {
        // l1 navigation logic breaks down when wind speed exceeds max airspeed
        // compute 2D groundspeed from airspeed-heading projection
        const Vector2f air_speed_2d{_airspeed * cosf(_yaw), _airspeed * sinf(_yaw)};

        // angle between air_speed_2d and ground_speed
        const float air_gnd_angle = acosf(
                (air_speed_2d * ground_speed) / (air_speed_2d.length() * ground_speed.length()));

        // if angle > 90 degrees or groundspeed is less than threshold, replace groundspeed with airspeed projection
        if ((fabsf(air_gnd_angle) > M_PI_2_F) || (ground_speed.length() < 3.0f)) {
            nav_speed_2d = air_speed_2d;
        }

//GIMBALL YAW LOGIC
        float air_nav_angle = acosf((air_speed_2d * nav_speed_2d) / (air_speed_2d.length() * nav_speed_2d.length()));
        float pwm_yaw;
        if ((air_speed_2d(0) * nav_speed_2d(1) - air_speed_2d(1) * nav_speed_2d(0)) > 0)
            pwm_yaw = air_nav_angle / (float)M_PI_4 * 2;
        else
            pwm_yaw = 0.0f - air_nav_angle / (float)M_PI_4 * 2;
        act1.control[4] = pwm_yaw;

        if (act_pub1 != nullptr) {
            orb_publish(ORB_ID(actuator_controls_1), act_pub1, &act1);
        } else {
            act_pub1 = orb_advertise(ORB_ID(actuator_controls_1), &act1);
        }
    }

    /* no throttle limit as default */
    float throttle_max = 1.0f;

    /* save time when airplane is in air */
    if (!_was_in_air && !_vehicle_land_detected.landed) {
        _was_in_air = true;
        _time_went_in_air = hrt_absolute_time();
        _takeoff_ground_alt = _global_pos.alt;
    }

    /* reset flag when airplane landed */
    if (_vehicle_land_detected.landed) {
        _was_in_air = false;
    }

    /* Reset integrators if switching to this mode from a other mode in which posctl was not active */
    if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
        /* reset integrators */
        _tecs.reset_state();
    }

    if (_manual_mode_enabled) {
        if (hrt_elapsed_time(&_manual_mode_last_updated) > 30e6) {
            mavlink_log_critical(&_mavlink_log_pub, "No updating manual control 30s, switching to auto");
            _manual_mode_enabled = false;

            set_mode(157, 4, 4);
        }
    }

    if (_control_mode.flag_control_auto_enabled && pos_sp_curr.valid) {
        /* AUTONOMOUS FLIGHT */

        _control_mode_current = FW_POSCTRL_MODE_AUTO;

        /* reset hold altitude */
        _hold_alt = _global_pos.alt;

        /* reset hold yaw */
        _hdg_hold_yaw = _yaw;

        /* get circle mode */
        bool was_circle_mode = _l1_control.circle_mode();

        /* restore TECS parameters, in case changed intermittently (e.g. in landing handling) */
        _tecs.set_speed_weight(_parameters.speed_weight);
        _tecs.set_time_const_throt(_parameters.time_const_throt);

        /* current waypoint (the one currently heading for) */
        Vector2f curr_wp((float) pos_sp_curr.lat, (float) pos_sp_curr.lon);

        /* Initialize attitude controller integrator reset flags to 0 */
        _att_sp.roll_reset_integral = false;
        _att_sp.pitch_reset_integral = false;
        _att_sp.yaw_reset_integral = false;

        /* previous waypoint */
        Vector2f prev_wp{0.0f, 0.0f};

        if (pos_sp_prev.valid) {
            prev_wp(0) = (float) pos_sp_prev.lat;
            prev_wp(1) = (float) pos_sp_prev.lon;

        } else {
            /*
             * No valid previous waypoint, go for the current wp.
             * This is automatically handled by the L1 library.
             */
            prev_wp(0) = (float) pos_sp_curr.lat;
            prev_wp(1) = (float) pos_sp_curr.lon;
        }

        float mission_airspeed = _parameters.airspeed_trim;

        if (PX4_ISFINITE(pos_sp_curr.cruising_speed) &&
            pos_sp_curr.cruising_speed > 0.1f) {

            mission_airspeed = pos_sp_curr.cruising_speed;
            _parameters.airspeed_trim = mission_airspeed;
            _parameters.airspeed_max = mission_airspeed + 1.f;
            _parameters.airspeed_min = mission_airspeed - 1.f;
        }

        float mission_throttle = _parameters.throttle_cruise;

        if (PX4_ISFINITE(pos_sp_curr.cruising_throttle) &&
            pos_sp_curr.cruising_throttle > 0.01f) {

            mission_throttle = pos_sp_curr.cruising_throttle;
        }

        if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
            _att_sp.thrust_body[0] = 0.0f;
            _att_sp.roll_body = 0.0f;
            _att_sp.pitch_body = 0.0f;

        } else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {

//TURNING LOOP LIGIC START//check if turning is needing
			// if (!do_turning_loop) {
			// 	const float dist_curr_prev = get_distance_to_next_waypoint(pos_sp_curr.lat, pos_sp_curr.lon, pos_sp_prev.lat, pos_sp_prev.lon);
			// 	if (dist_curr_prev < 150) {
			// 		mavlink_log_critical(&_mavlink_log_pub, "Start turning loop");
            //         param_get(_parameter_handles.loiter_radius, &radius);
			// 		do_turning_loop = true;

			// 		double curr_next_lat = 0;
			// 		double curr_next_lon = 0;

			// 		double curr_prev_lat = 0;
			// 		double curr_prev_lon = 0;

			// 		//pre_enter wp
			// 		loop_pre_enter_lat = pos_sp_prev.lat;
			// 		loop_pre_enter_lon = pos_sp_prev.lon;
			// 		acc_turning_radius = 10;

			// 		//enter wp
			// 		create_waypoint_from_line_and_dist(pos_sp_curr.lat, pos_sp_curr.lon,
			// 					pos_sp_next.lat, pos_sp_next.lon, radius, &curr_next_lat, &curr_next_lon);
			// 		create_waypoint_from_line_and_dist(pos_sp_curr.lat, pos_sp_curr.lon,
			// 					pos_sp_prev.lat, pos_sp_prev.lon, radius*0.6f, &curr_prev_lat, &curr_prev_lon);
			// 		create_waypoint_from_line_and_dist(curr_prev_lat, curr_prev_lon,
			// 					curr_next_lat, curr_next_lon, -radius*1.1f, &loop_enter_lat, &loop_enter_lon);

			// 		//middle wp
			// 		create_waypoint_from_line_and_dist(pos_sp_curr.lat, pos_sp_curr.lon,
			// 					pos_sp_next.lat, pos_sp_next.lon, -radius, &loop_middle_lat, &loop_middle_lon);

			// 		//pre_exit wp
			// 		create_waypoint_from_line_and_dist(pos_sp_curr.lat, pos_sp_curr.lon,
			// 					pos_sp_next.lat, pos_sp_next.lon, -radius*0.2f, &loop_pre_exit_lat, &loop_pre_exit_lon);

			// 		//exit wp
			// 		loop_exit_lat = pos_sp_curr.lat;
			// 		loop_exit_lon = pos_sp_curr.lon;

			// 	}
			// }

			// if (do_turning_loop){
			// 	switch(loop_waypoint_curr){
			// 		case 0:
			// 			curr_wp = Vector2f((float)loop_pre_enter_lat, (float)loop_pre_enter_lon);
			// 			break;
			// 		case 1:
			// 			curr_wp = Vector2f((float)loop_enter_lat, (float)loop_enter_lon);
			// 			break;
			// 		case 2:
			// 			//moving towards middle point
			// 			prev_wp = Vector2f((float)loop_enter_lat, (float)loop_enter_lon);
			// 			curr_wp = Vector2f((float)loop_middle_lat, (float)loop_middle_lon);
			// 			break;

			// 		case 3:
			// 			acc_turning_radius = radius * 0.3f;
			// 			prev_wp = Vector2f((float)loop_middle_lat, (float)loop_middle_lon);
			// 			curr_wp = Vector2f((float)loop_pre_exit_lat, (float)loop_pre_exit_lon);
			// 			break;
			// 		case 4:
			// 			prev_wp = Vector2f((float)loop_pre_exit_lat, (float)loop_pre_exit_lon);
			// 			curr_wp = Vector2f((float)loop_exit_lat, (float)loop_exit_lon);
			// 			break;
			// 		default:
			// 			break;
			// 	}


			// 	const float dist_xy = get_distance_to_next_waypoint(curr_pos(0), curr_pos(1), curr_wp(0), curr_wp(1));
			// 	//mavlink_log_critical(&_mavlink_log_pub, "distance = %f", dist_xy);
			// 	if (dist_xy < acc_turning_radius) {
			// 		loop_waypoint_curr ++;
			// 		acc_turning_radius = radius * 0.6f;
			// 		mavlink_log_critical(&_mavlink_log_pub, "Turning wp %d", loop_waypoint_curr);
			// 		if (loop_waypoint_curr >= 5){
			// 			mavlink_log_critical(&_mavlink_log_pub, "Exit turning loop");
			// 			curr_wp = Vector2f((float)pos_sp_curr.lat, (float)pos_sp_curr.lon);
			// 			prev_wp = Vector2f((float)loop_pre_exit_lat, (float)loop_exit_lon);
			// 			loop_waypoint_curr = 0;
			// 			do_turning_loop = false;
			// 		}
			// 	}
			// }
//TURNING END
            /* waypoint is a plain navigation waypoint */
            _l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, nav_speed_2d);
            _att_sp.roll_body = _l1_control.get_roll_setpoint();
            _att_sp.yaw_body = _l1_control.nav_bearing();

            float alt = abs(pos_sp_curr.alt - pos_sp_prev.alt) > 50 ? pos_sp_prev.alt : pos_sp_curr.alt;

            tecs_update_pitch_throttle(alt,
                                       calculate_target_airspeed(mission_airspeed),
                                       radians(_parameters.pitch_limit_min) - _parameters.pitchsp_offset_rad,
                                       radians(_parameters.pitch_limit_max) - _parameters.pitchsp_offset_rad,
                                       _airspeed_valid ? _parameters.throttle_min : _parameters.throttle_cruise,
                                       _parameters.throttle_max,            //may be min?
                                       mission_throttle,                    // may be min?
                                       false,
                                       radians(_parameters.pitch_limit_min));

        } else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

            /* waypoint is a loiter waypoint */
            _l1_control.navigate_loiter(curr_wp, curr_pos, pos_sp_curr.loiter_radius,
                                        pos_sp_curr.loiter_direction, nav_speed_2d);
            _att_sp.roll_body = _l1_control.get_roll_setpoint();
            _att_sp.yaw_body = _l1_control.nav_bearing();

            float alt_sp = pos_sp_curr.alt;

            if (pos_sp_next.type == position_setpoint_s::SETPOINT_TYPE_LAND && pos_sp_next.valid
                && _l1_control.circle_mode() && _parameters.land_early_config_change == 1) {
                // We're in a loiter directly before a landing WP. Enable our landing configuration (flaps,
                // landing airspeed and potentially tighter throttle control) already such that we don't
                // have to do this switch (which can cause significant altitude errors) close to the ground.
                _tecs.set_time_const_throt(_parameters.land_throtTC_scale * _parameters.time_const_throt);
                mission_airspeed = _parameters.land_airspeed_scale * _parameters.airspeed_min;
                _att_sp.apply_flaps = true;
            }

            if (in_takeoff_situation()) {
                alt_sp = max(alt_sp, _takeoff_ground_alt + _parameters.climbout_diff);
                _att_sp.roll_body = constrain(_att_sp.roll_body, radians(-5.0f), radians(5.0f));
            }

            if (_land_abort) {
                if (pos_sp_curr.alt - _global_pos.alt < _parameters.climbout_diff) {
                    // aborted landing complete, normal loiter over landing point
                    abort_landing(false);

                } else {
                    // continue straight until vehicle has sufficient altitude
                    _att_sp.roll_body = 0.0f;
                }

                _tecs.set_time_const_throt(_parameters.land_throtTC_scale * _parameters.time_const_throt);
            }

            tecs_update_pitch_throttle(alt_sp,
                                       calculate_target_airspeed(mission_airspeed),
                                       radians(_parameters.pitch_limit_min) - _parameters.pitchsp_offset_rad,
                                       radians(_parameters.pitch_limit_max) - _parameters.pitchsp_offset_rad,
                                       _parameters.throttle_min,
                                       _parameters.throttle_max,
                                       _parameters.throttle_cruise,
                                       false,
                                       radians(_parameters.pitch_limit_min));

        } else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
            /* waypoint is a plain navigation waypoint */
            _l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, nav_speed_2d);
            _att_sp.roll_body = _l1_control.get_roll_setpoint();
            _att_sp.yaw_body = _l1_control.nav_bearing();

            float wp_distance = get_distance_to_next_waypoint((double) curr_pos(0), (double) curr_pos(1), (double) curr_wp(0),
                                                      (double) curr_wp(1));
            control_landing(curr_pos, ground_speed, pos_sp_prev, pos_sp_curr, wp_distance);

        } else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
            control_takeoff(curr_pos, ground_speed, pos_sp_prev, pos_sp_curr);
        }

        /* reset landing state */
        if (pos_sp_curr.type != position_setpoint_s::SETPOINT_TYPE_LAND) {
            reset_landing_state();
        }

        /* reset takeoff/launch state */
        if (pos_sp_curr.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
            reset_takeoff_state();
        }


        if (was_circle_mode && !_l1_control.circle_mode()) {
            /* just kicked out of loiter, reset roll integrals */
            _att_sp.roll_reset_integral = true;
        }

    } else if (_control_mode.flag_control_velocity_enabled &&
               _control_mode.flag_control_altitude_enabled) {
        /* POSITION CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed,
           heading is set to a distant waypoint */

        if (_control_mode_current != FW_POSCTRL_MODE_POSITION) {
            /* Need to init because last loop iteration was in a different mode */
            _hold_alt = _global_pos.alt;
            _hdg_hold_yaw = _yaw;
            _hdg_hold_enabled = false; // this makes sure the waypoints are reset below
            _yaw_lock_engaged = false;

            /* reset setpoints from other modes (auto) otherwise we won't
             * level out without new manual input */
            _att_sp.roll_body = 0;
            _att_sp.yaw_body = 0;

            _manual_mode_last_updated = hrt_absolute_time();
            _manual_mode_enabled = true;
        }

        _control_mode_current = FW_POSCTRL_MODE_POSITION;

        float altctrl_airspeed = _parameters.airspeed_trim;

        /* update desired altitude based on user pitch stick input */
        bool climbout_requested = update_desired_altitude(dt);

        /* throttle limiting */
        throttle_max = _parameters.throttle_max;

        if (_vehicle_land_detected.landed) {
            throttle_max = 0.0f;
        }

        tecs_update_pitch_throttle(_hold_alt,
                                   altctrl_airspeed,
                                   radians(_parameters.pitch_limit_min),
                                   radians(_parameters.pitch_limit_max),
                                   _parameters.throttle_min,
                                   throttle_max,
                                   _parameters.throttle_cruise,
                                   climbout_requested,
                                   climbout_requested ? radians(10.0f) : _parameters.pitch_limit_min,
                                   tecs_status_s::TECS_MODE_NORMAL);

        _hdg_hold_yaw = radians(_manual.r);
        get_waypoint_heading_distance(_hdg_hold_yaw, _hdg_hold_prev_wp, _hdg_hold_curr_wp, true);

        Vector2f prev_wp{(float) _hdg_hold_prev_wp.lat, (float) _hdg_hold_prev_wp.lon};
        Vector2f curr_wp{(float) _hdg_hold_curr_wp.lat, (float) _hdg_hold_curr_wp.lon};

        /* populate l1 control setpoint */
        _l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);

        _att_sp.roll_body = _l1_control.get_roll_setpoint();
        _att_sp.yaw_body = _l1_control.nav_bearing();

    } else if (_control_mode.flag_control_altitude_enabled) {
        /* ALTITUDE CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed */

        if (_control_mode_current != FW_POSCTRL_MODE_POSITION && _control_mode_current != FW_POSCTRL_MODE_ALTITUDE) {
            /* Need to init because last loop iteration was in a different mode */
            _hold_alt = _global_pos.alt;
            _manual_mode_last_updated = hrt_absolute_time();
            _manual_mode_enabled = true;
            mavlink_log_critical(&_mavlink_log_pub, "manual control enabled");
        }

        _control_mode_current = FW_POSCTRL_MODE_ALTITUDE;

        /* Get demanded airspeed */
        float altctrl_airspeed = _parameters.airspeed_trim;

        /* update desired altitude based on user pitch stick input */
        bool climbout_requested = update_desired_altitude(dt);

        /* throttle limiting */
        throttle_max = _parameters.throttle_max;

        if (_vehicle_land_detected.landed) {
            throttle_max = 0.0f;
        }

        tecs_update_pitch_throttle(_hold_alt,
                                   altctrl_airspeed,
                                   radians(_parameters.pitch_limit_min),
                                   radians(_parameters.pitch_limit_max),
                                   _parameters.throttle_min,
                                   throttle_max,
                                   _parameters.throttle_cruise,
                                   climbout_requested,
                                   climbout_requested ? radians(10.0f) : _parameters.pitch_limit_min,
                                   tecs_status_s::TECS_MODE_NORMAL);

        _att_sp.roll_body = radians(_manual.y);
        _att_sp.yaw_body = 0;

    } else {
        _control_mode_current = FW_POSCTRL_MODE_OTHER;

        /* do not publish the setpoint */
        setpoint = false;

        // reset hold altitude
        _hold_alt = _global_pos.alt;

        /* reset landing and takeoff state */
        if (!_last_manual) {
            reset_landing_state();
            reset_takeoff_state();
        }
    }

    /* Copy thrust output for publication */
    if (_control_mode_current == FW_POSCTRL_MODE_AUTO && // launchdetector only available in auto
        pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
        _launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS &&
        !_runway_takeoff.runwayTakeoffEnabled()) {

        /* making sure again that the correct thrust is used,
         * without depending on library calls for safety reasons.
           the pre-takeoff throttle and the idle throttle normally map to the same parameter. */
        _att_sp.thrust_body[0] = _parameters.throttle_idle;

    } else if (_control_mode_current == FW_POSCTRL_MODE_AUTO &&
               pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
               _runway_takeoff.runwayTakeoffEnabled()) {

        _att_sp.thrust_body[0] = _runway_takeoff.getThrottle(min(get_tecs_thrust(), throttle_max));

    } else if (_control_mode_current == FW_POSCTRL_MODE_AUTO &&
               pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

        _att_sp.thrust_body[0] = 0.0f;

    } else if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
        _att_sp.thrust_body[0] = min(_att_sp.thrust_body[0], _parameters.throttle_max);

    } else {
        /* Copy thrust and pitch values from tecs */
        if (_vehicle_land_detected.landed) {
            // when we are landed state we want the motor to spin at idle speed
            _att_sp.thrust_body[0] = min(_parameters.throttle_idle, throttle_max);
        } else {
            _att_sp.thrust_body[0] = min(get_tecs_thrust(), throttle_max);
        }
    }

    detect_unexpected_descent(pos_sp_curr);

    if (unexpected_descent){
        _att_sp.thrust_body[0] = 0.f;
        if (hrt_elapsed_time(&unexp_desc_time) > 2e6) {
            release_parachute();
            play_tune(11);
            set_mode();
            set_arm(false);
        }
    }

    // decide when to use pitch setpoint from TECS because in some cases pitch
    // setpoint is generated by other means
    bool use_tecs_pitch = true;

    // auto runway takeoff
    use_tecs_pitch &= !(_control_mode_current == FW_POSCTRL_MODE_AUTO &&
                        pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
                        (_launch_detection_state == LAUNCHDETECTION_RES_NONE ||
                         _runway_takeoff.runwayTakeoffEnabled()));

    // flaring during landing
    use_tecs_pitch &= !(pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LAND && _land_noreturn_vertical);

    // manual attitude control
    use_tecs_pitch &= !(_control_mode_current == FW_POSCTRL_MODE_OTHER);

    if (use_tecs_pitch) {
        _att_sp.pitch_body = get_tecs_pitch();
    }

    if (_control_mode.flag_control_position_enabled) {
        _last_manual = false;

    } else {
        _last_manual = true;
    }

    return setpoint;
}

void
FixedwingPositionControl::detect_unexpected_descent(position_setpoint_s pos_sp_curr){
    if (!check_unexp_desc){
        float diff = pos_sp_curr.alt - _global_pos.alt;
        if (diff > 50.f){
            check_unexp_desc = true;
            dangerous_diff = diff;
            dangerous_dist_to_takeoff_alt = _global_pos.alt - _takeoff_ground_alt;
            dang_alt_time_det = hrt_absolute_time();
        }
    } else {
        if (hrt_elapsed_time(&dang_alt_time_det) > 3e6) {
            float diff = pos_sp_curr.alt - _global_pos.alt;
            float curr_dist_to_takeoff_alt = _global_pos.alt - _takeoff_ground_alt;
            if (((diff - dangerous_diff) > 25) && (
                (dangerous_dist_to_takeoff_alt - curr_dist_to_takeoff_alt) > 25)){
                //detected an unexpected descent
                unexpected_descent = true;
                unexp_desc_time = hrt_absolute_time();
                mavlink_log_critical(&_mavlink_log_pub, "Unexpected descent %fm/s", (diff - dangerous_diff) / 3.f);
            } else
            {
                check_unexp_desc = false;
            }

        }

    }
}

void FixedwingPositionControl::set_mode(float base_mode, float main_mode, float custom_mode){

    vehicle_command_s vcmd_mode = {};
    vcmd_mode.timestamp = hrt_absolute_time();

    /* copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
    vcmd_mode.param1 = base_mode;
    vcmd_mode.param2 = main_mode;
    vcmd_mode.param3 = custom_mode;

    vcmd_mode.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
    vcmd_mode.target_system = 1;
    vcmd_mode.target_component = 1;
    vcmd_mode.source_system = 255;
    vcmd_mode.source_component = 0;
    vcmd_mode.confirmation = 0;
    vcmd_mode.from_external = true;

    orb_advertise_queue(ORB_ID(vehicle_command), &vcmd_mode, vehicle_command_s::ORB_QUEUE_LENGTH);
}

void FixedwingPositionControl::set_arm(bool enable){
    vehicle_command_s vcmd_disarm = {};
    vcmd_disarm.timestamp = hrt_absolute_time();
    vcmd_disarm.param1 = enable ? 1 : 0;
    vcmd_disarm.param2 = 0;
    vcmd_disarm.param3 = 0;
    vcmd_disarm.param4 = 0;
    vcmd_disarm.param5 = 0;
    vcmd_disarm.param6 = 0;
    vcmd_disarm.param7 = 0;
    vcmd_disarm.command = 400;
    vcmd_disarm.target_system = 1;
    vcmd_disarm.target_component = 1;
    vcmd_disarm.source_system = 255;
    vcmd_disarm.source_component = 0;
    vcmd_disarm.confirmation = 0;
    vcmd_disarm.from_external = true;

    orb_advertise_queue(ORB_ID(vehicle_command), &vcmd_disarm, vehicle_command_s::ORB_QUEUE_LENGTH);
}

void
FixedwingPositionControl::play_tune(uint8_t id){
    tune_control_s tc = {};
    tc.tune_id = id;
    tc.volume = tune_control_s::VOLUME_LEVEL_MAX;
    tc.tune_override = 0;
    tc.timestamp = hrt_absolute_time();
    orb_advertise(ORB_ID(tune_control), &tc);
}

void
FixedwingPositionControl::release_parachute(){
     if (_parameters.sys_autostart == 3239) {
        act1.control[5] = 0.65f;
    }else if (_parameters.sys_autostart == 2101) {
        act1.control[5] = -0.97f; //parachute drop
        act1.control[6] = 0.15f; //buffer drop
    }
    act1.timestamp = hrt_absolute_time();
    if (act_pub1 != nullptr) {
        orb_publish(ORB_ID(actuator_controls_1), act_pub1, &act1);
    } else {
        act_pub1 = orb_advertise(ORB_ID(actuator_controls_1), &act1);
    }
    mavlink_log_critical(&_mavlink_log_pub, "Parachute is released");
}

void
FixedwingPositionControl::drop_parachute(){
    if (_parameters.sys_autostart == 3239){
        act1.control[5] = 0.9f;
    } else if (_parameters.sys_autostart == 2101) {
        act1.control[7] = 1.0f;
        act1.control[6] = 0.0;
    } else {
        mavlink_log_critical(&_mavlink_log_pub, "Unsupported airframe");
    }
    act1.timestamp = hrt_absolute_time();
    if (act_pub1 != nullptr) {
        orb_publish(ORB_ID(actuator_controls_1), act_pub1, &act1);
    } else {
        act_pub1 = orb_advertise(ORB_ID(actuator_controls_1), &act1);
    }
    mavlink_log_critical(&_mavlink_log_pub, "Parachute is dropped");
}

void
FixedwingPositionControl::control_takeoff(const Vector2f &curr_pos, const Vector2f &ground_speed,
                                          const position_setpoint_s &pos_sp_prev,
                                          const position_setpoint_s &pos_sp_curr) {
    /* current waypoint (the one currently heading for) */
    Vector2f curr_wp((float) pos_sp_curr.lat, (float) pos_sp_curr.lon);
    Vector2f prev_wp{0.0f, 0.0f}; /* previous waypoint */

    if (pos_sp_prev.valid) {
        prev_wp(0) = (float) pos_sp_prev.lat;
        prev_wp(1) = (float) pos_sp_prev.lon;

    } else {
        /*
         * No valid previous waypoint, go for the current wp.
         * This is automatically handled by the L1 library.
         */
        prev_wp(0) = (float) pos_sp_curr.lat;
        prev_wp(1) = (float) pos_sp_curr.lon;
    }

    // apply flaps for takeoff according to the corresponding scale factor set
    // via FW_FLAPS_TO_SCL
    _att_sp.apply_flaps = vehicle_attitude_setpoint_s::FLAPS_TAKEOFF;

    // continuously reset launch detection and runway takeoff until armed
    if (!_control_mode.flag_armed) {
        _launchDetector.reset();
        _launch_detection_state = LAUNCHDETECTION_RES_NONE;
        _launch_detection_notify = 0;
    }


    /* Perform launch detection */
    if (_launchDetector.launchDetectionEnabled() &&
        _launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {

        if (_control_mode.flag_armed) {
            /* Perform launch detection */

            // create virtual waypoint in 500m ahead before takeoff
            Eulerf euler(Quatf(_att.q));
            get_waypoint_heading_distance(euler.psi(), _hdg_hold_prev_wp, _hdg_hold_curr_wp, true);
            _takeoff_ground_alt = _global_pos.alt;

            /* Inform user that launchdetection is running every 4s */
            if (hrt_elapsed_time(&_launch_detection_notify) > 4e6) {
                mavlink_log_critical(&_mavlink_log_pub, "Launch detection running 010");
                _launch_detection_notify = hrt_absolute_time();
            }

            /* Detect launch using body X (forward) acceleration */
            _launchDetector.update(_sub_sensors.get().accel_x);

            /* update our copy of the launch detection state */
            _launch_detection_state = _launchDetector.getLaunchDetected();
        }

    } else {
        /* no takeoff detection --> fly */
        _launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
    }

    /* Set control values depending on the detection state */
    if (_launch_detection_state != LAUNCHDETECTION_RES_NONE) {
        /* Launch has been detected, hence we have to control the plane. */

        /* Select throttle: only in LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS we want to use
            * full throttle, otherwise we use idle throttle */
        float takeoff_throttle = _parameters.throttle_max;

        if (_launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
            takeoff_throttle = _parameters.throttle_idle;
        }

        /* select maximum pitch: the launchdetector may impose another limit for the pitch
            * depending on the state of the launch */
        const float takeoff_pitch_max_deg = _launchDetector.getPitchMax(_parameters.pitch_limit_max);
        const float altitude_diff = _global_pos.alt - _takeoff_ground_alt;

        /* apply minimum pitch and limit roll if target altitude is not within climbout_diff meters */
        if (!climbout_completed && _parameters.climbout_diff > 0.0f && altitude_diff < _parameters.climbout_diff) {
            /* enforce a minimum of 10 degrees pitch up on takeoff, or take parameter */

            Vector2f prev_wp_takeoff{(float) _hdg_hold_prev_wp.lat, (float) _hdg_hold_prev_wp.lon};
            Vector2f curr_wp_takeoff{(float) _hdg_hold_curr_wp.lat, (float) _hdg_hold_curr_wp.lon};

            /* populate l1 control setpoint */
            _l1_control.navigate_waypoints(prev_wp_takeoff, curr_wp_takeoff, curr_pos, ground_speed);

            _att_sp.roll_body = _l1_control.get_roll_setpoint();
            _att_sp.yaw_body = _l1_control.nav_bearing();
            takeoff_throttle = 1.0f;
            tecs_update_pitch_throttle(pos_sp_curr.alt,
                                        _parameters.airspeed_trim,
                                        radians(10.0f),
                                        radians(takeoff_pitch_max_deg),
                                        _parameters.throttle_min,
                                        takeoff_throttle,
                                        _parameters.throttle_cruise,
                                        true,
                                        max(radians(pos_sp_curr.pitch_min), radians(20.0f)),
                                        tecs_status_s::TECS_MODE_TAKEOFF);



            // /* limit roll motion to ensure enough lift */
            _att_sp.roll_body = constrain(_att_sp.roll_body, radians(-10.0f), radians(10.0f));

        } else {
            climbout_completed = true;

            _l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);
            _att_sp.roll_body = _l1_control.get_roll_setpoint();
            _att_sp.yaw_body = _l1_control.nav_bearing();

            tecs_update_pitch_throttle(pos_sp_curr.alt,
                                        calculate_target_airspeed(_parameters.airspeed_trim),
                                        radians(_parameters.pitch_limit_min),
                                        radians(_parameters.pitch_limit_max),
                                        _parameters.throttle_min,
                                        takeoff_throttle,
                                        _parameters.throttle_cruise,
                                        false,
                                        radians(_parameters.pitch_limit_min));
        }

    } else {
        /* Tell the attitude controller to stop integrating while we are waiting
            * for the launch */
        _att_sp.roll_reset_integral = true;
        _att_sp.pitch_reset_integral = true;
        _att_sp.yaw_reset_integral = true;

        /* Set default roll and pitch setpoints during detection phase */
        _att_sp.roll_body = 0.0f;
        _att_sp.pitch_body = max(radians(pos_sp_curr.pitch_min), radians(10.0f));
    }
}

void
FixedwingPositionControl::control_landing(const Vector2f &curr_pos, const Vector2f &ground_speed,
                                              const position_setpoint_s &pos_sp_prev,
                                              const position_setpoint_s &pos_sp_curr, float wp_distance) {

    const float airspeed_land = _parameters.airspeed_min;

    float throttle_land = 0.f;
    float throttle_max = 0.f;
    float throttle_min = 0.f;

    if (wp_distance > 80.f){
        throttle_land = _parameters.throttle_cruise;
        throttle_max = _parameters.throttle_max;
        throttle_min = _parameters.throttle_min;
    } else if (wp_distance > 50.f) {
        throttle_land = _parameters.throttle_min;
        throttle_max = _parameters.throttle_min;
        throttle_min = _parameters.throttle_min;
    } else if (wp_distance > 20.f) {
        _land_motor_lim = true;
        throttle_land = 0.f;
        throttle_max = 0.f;
        throttle_min = 0.f;
    } else {

        if (!parachute_released){
            release_parachute();
        }

        parachute_released = true;

    }
    if (parachute_released){
        throttle_land = 0.f;
        throttle_max = 0.f;
        throttle_min = 0.f;
    }

    if (!parachute_dropped && _vehicle_land_detected.landed) {
        drop_parachute();
        parachute_dropped = true;

        play_tune(11);
        set_mode();
        set_arm(false);

    }

    tecs_update_pitch_throttle(pos_sp_curr.alt,
                               airspeed_land,
                               radians(_parameters.pitch_limit_min) - _parameters.pitchsp_offset_rad,
                               radians(_parameters.pitch_limit_max) - _parameters.pitchsp_offset_rad,
                               throttle_min,
                               throttle_max,
                               throttle_land,
                               false,
                               radians(_parameters.pitch_limit_min),
                               _land_motor_lim ? tecs_status_s::TECS_MODE_LAND_THROTTLELIM
                                               : tecs_status_s::TECS_MODE_LAND);


}

float
FixedwingPositionControl::get_tecs_pitch() {
    if (_is_tecs_running) {
        return _tecs.get_pitch_setpoint();
    }

    // return 0 to prevent stale tecs state when it's not running
    return 0.0f;
}

float
FixedwingPositionControl::get_tecs_thrust() {
    if (_is_tecs_running) {
        return _tecs.get_throttle_setpoint();
    }

    // return 0 to prevent stale tecs state when it's not running
    return 0.0f;
}

void
FixedwingPositionControl::handle_command() {
    if (_vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {
        // only abort landing before point of no return (horizontal and vertical)
        if (_control_mode.flag_control_auto_enabled &&
            _pos_sp_triplet.current.valid &&
            _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

            abort_landing(true);
        }
    }
}

void
FixedwingPositionControl::run() {
    /*
     * do subscriptions
     */
    _global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
    _control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    _sensor_baro_sub = orb_subscribe(ORB_ID(sensor_baro));

    /* rate limit position updates to 50 Hz */
    orb_set_interval(_global_pos_sub, 20);

    /* abort on a nonzero return value from the parameter init */
    if (parameters_update() != PX4_OK) {
        /* parameter setup went wrong, abort */
        PX4_ERR("aborting startup due to errors.");
    }

    /* wakeup source(s) */
    px4_pollfd_struct_t fds[1];

    /* Setup of loop */
    fds[0].fd = _global_pos_sub;
    fds[0].events = POLLIN;

    while (!should_exit()) {

        /* wait for up to 500ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

        // /* timed out - periodic check for _task_should_exit, etc. */
        // if (pret == 0) {
        //     continue;
        // }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
            PX4_WARN("poll error %d, %d", pret, errno);
            continue;
        }

        /* only update parameters if they changed */
        bool params_updated = false;
        orb_check(_params_sub, &params_updated);

        if (params_updated) {
            /* read from param to clear updated flag */
            parameter_update_s update;
            orb_copy(ORB_ID(parameter_update), _params_sub, &update);

            /* update parameters from storage */
            parameters_update();
        }

        /* always run controller */
        if (true || (fds[0].revents & POLLIN) != 0) {
            perf_begin(_loop_perf);

            /* load local copies */
            orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
            orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

            // handle estimator reset events. we only adjust setpoins for manual modes
            if (_control_mode.flag_control_manual_enabled) {
                if (_control_mode.flag_control_altitude_enabled &&
                    _global_pos.alt_reset_counter != _alt_reset_counter) {
                    _hold_alt += _global_pos.delta_alt;
                    // make TECS accept step in altitude and demanded altitude
                    _tecs.handle_alt_step(_global_pos.delta_alt, _global_pos.alt);
                }

                // adjust navigation waypoints in position control mode
                if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
                    && _global_pos.lat_lon_reset_counter != _pos_reset_counter) {

                    // reset heading hold flag, which will re-initialise position control
                    _hdg_hold_enabled = false;
                }
            }

            // update the reset counters in any case
            _alt_reset_counter = _global_pos.alt_reset_counter;
            _pos_reset_counter = _global_pos.lat_lon_reset_counter;

            _sub_sensors.update();
            airspeed_poll();
            manual_control_setpoint_poll();
            position_setpoint_triplet_poll();
            vehicle_attitude_poll();
            vehicle_command_poll();
            vehicle_control_mode_poll();
            vehicle_land_detected_poll();
            vehicle_status_poll();

            Vector2f curr_pos((float) _global_pos.lat, (float) _global_pos.lon);
            Vector2f ground_speed(_global_pos.vel_n, _global_pos.vel_e);

            if (pret == 0 && !_control_mode.flag_control_altitude_enabled ) {
                continue;
            }

            /*
            * Attempt to control position, on success (= sensors present and not in manual mode),
            * publish setpoint.
            */
            if (control_position(curr_pos, ground_speed, _pos_sp_triplet.previous, _pos_sp_triplet.current,
                                 _pos_sp_triplet.next)) {
                _att_sp.timestamp = hrt_absolute_time();

                // add attitude setpoint offsets
                _att_sp.roll_body += _parameters.rollsp_offset_rad;
                _att_sp.pitch_body += _parameters.pitchsp_offset_rad;

                if (_control_mode.flag_control_manual_enabled) {
                    _att_sp.roll_body = constrain(_att_sp.roll_body, -_parameters.man_roll_max_rad,
                                                  _parameters.man_roll_max_rad);
                    _att_sp.pitch_body = constrain(_att_sp.pitch_body, -_parameters.man_pitch_max_rad,
                                                   _parameters.man_pitch_max_rad);
                }

                Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
                q.copyTo(_att_sp.q_d);
                _att_sp.q_d_valid = true;

                if (!_control_mode.flag_control_offboard_enabled ||
                    _control_mode.flag_control_position_enabled ||
                    _control_mode.flag_control_velocity_enabled ||
                    _control_mode.flag_control_acceleration_enabled) {

                    /* lazily publish the setpoint only once available */
                    if (_attitude_sp_pub != nullptr) {
                        /* publish the attitude setpoint */
                        orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &_att_sp);

                    } else if (_attitude_setpoint_id != nullptr) {
                        /* advertise and publish */
                        _attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
                    }

                    // only publish status in full FW mode
                    if (!_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode) {
                        status_publish();
                    }
                }
            }

            perf_end(_loop_perf);
        }
    }
}

void
FixedwingPositionControl::reset_takeoff_state(bool force) {
    // only reset takeoff if !armed or just landed
    if (!_control_mode.flag_armed || (_was_in_air && _vehicle_land_detected.landed) || force) {

        _runway_takeoff.reset();

        _launchDetector.reset();
        _launch_detection_state = LAUNCHDETECTION_RES_NONE;
        _launch_detection_notify = 0;

    } else {
        _launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
    }
}

void
FixedwingPositionControl::reset_landing_state() {
    _time_started_landing = 0;

    // reset terrain estimation relevant values
    _time_last_t_alt = 0;

    _land_noreturn_horizontal = false;
    _land_noreturn_vertical = false;
    _land_stayonground = false;
    _land_motor_lim = false;
    _land_onslope = false;

    // reset abort land, unless loitering after an abort
    if (_land_abort && (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LOITER)) {

        abort_landing(false);
    }
}

void
FixedwingPositionControl::tecs_update_pitch_throttle(float alt_sp, float airspeed_sp,
                                                     float pitch_min_rad, float pitch_max_rad,
                                                     float throttle_min, float throttle_max, float throttle_cruise,
                                                     bool climbout_mode, float climbout_pitch_min_rad,
                                                     uint8_t mode) {

    float dt = 0.01f; // prevent division with 0

    if (_last_tecs_update > 0) {
        dt = hrt_elapsed_time(&_last_tecs_update) * 1e-6;
    }

    _last_tecs_update = hrt_absolute_time();

    // do not run TECS if we are not in air
    bool run_tecs = !_vehicle_land_detected.landed;

    // do not run TECS if vehicle is a VTOL and we are in rotary wing mode or in transition
    // (it should also not run during VTOL blending because airspeed is too low still)
    if (_vehicle_status.is_vtol) {
        if (_vehicle_status.is_rotary_wing || _vehicle_status.in_transition_mode) {
            run_tecs = false;
        }

        if (_vehicle_status.in_transition_mode) {
            // we're in transition
            _was_in_transition = true;

            // set this to transition airspeed to init tecs correctly
            if (_parameters.airspeed_disabled && PX4_ISFINITE(_parameters.airspeed_trans)) {
                // some vtols fly without airspeed sensor
                _asp_after_transition = _parameters.airspeed_trans;

            } else {
                _asp_after_transition = _airspeed;
            }

            _asp_after_transition = constrain(_asp_after_transition, _parameters.airspeed_min,
                                              _parameters.airspeed_max);

        } else if (_was_in_transition) {
            // after transition we ramp up desired airspeed from the speed we had coming out of the transition
            _asp_after_transition += dt * 2; // increase 2m/s

            if (_asp_after_transition < airspeed_sp && _airspeed < airspeed_sp) {
                airspeed_sp = max(_asp_after_transition, _airspeed);

            } else {
                _was_in_transition = false;
                _asp_after_transition = 0;
            }
        }
    }

    _is_tecs_running = run_tecs;

    if (!run_tecs) {
        // next time we run TECS we should reinitialize states
        _reinitialize_tecs = true;
        return;
    }

    if (_reinitialize_tecs) {
        _tecs.reset_state();
        _reinitialize_tecs = false;
    }

    if (_vehicle_status.engine_failure) {
        /* Force the slow downwards spiral */
        pitch_min_rad = M_DEG_TO_RAD_F * -1.0f;
        pitch_max_rad = M_DEG_TO_RAD_F * 5.0f;
    }

    /* No underspeed protection in landing mode */
    _tecs.set_detect_underspeed_enabled(!(mode == tecs_status_s::TECS_MODE_LAND
                                          || mode == tecs_status_s::TECS_MODE_LAND_THROTTLELIM));

    /* Using tecs library */
    float pitch_for_tecs = _pitch - _parameters.pitchsp_offset_rad;

    /* filter speed and altitude for controller */
    Vector3f accel_body(_sub_sensors.get().accel_x, _sub_sensors.get().accel_y, _sub_sensors.get().accel_z);

    // tailsitters use the multicopter frame as reference, in fixed wing
    // we need to use the fixed wing frame
    if (_parameters.vtol_type == vtol_type::TAILSITTER && _vehicle_status.is_vtol) {
        float tmp = accel_body(0);
        accel_body(0) = -accel_body(2);
        accel_body(2) = tmp;
    }

    /* tell TECS to update its state, but let it know when it cannot actually control the plane */
    bool in_air_alt_control = (!_vehicle_land_detected.landed &&
                               (_control_mode.flag_control_auto_enabled ||
                                _control_mode.flag_control_velocity_enabled ||
                                _control_mode.flag_control_altitude_enabled));

    /* update TECS vehicle state estimates */
    _tecs.update_vehicle_state_estimates(_airspeed, _R_nb,
                                         accel_body, (_global_pos.timestamp > 0), in_air_alt_control,
                                         _global_pos.alt, _local_pos.v_z_valid, _local_pos.vz, _local_pos.az);

    /* scale throttle cruise by baro pressure */
    if (_parameters.throttle_alt_scale > FLT_EPSILON) {

        bool baro_updated = false;
        orb_check(_sensor_baro_sub, &baro_updated);

        sensor_baro_s baro;

        if (orb_copy(ORB_ID(sensor_baro), _sensor_baro_sub, &baro) == PX4_OK) {
            if (PX4_ISFINITE(baro.pressure) && PX4_ISFINITE(_parameters.throttle_alt_scale)) {
                // scale throttle as a function of sqrt(p0/p) (~ EAS -> TAS at low speeds and altitudes ignoring temperature)
                const float eas2tas = sqrtf(CONSTANTS_STD_PRESSURE_MBAR / baro.pressure);
                const float scale = constrain((eas2tas - 1.0f) * _parameters.throttle_alt_scale + 1.0f, 1.0f, 2.0f);

                throttle_max = constrain(throttle_max * scale, throttle_min, 1.0f);
                throttle_cruise = constrain(throttle_cruise * scale, throttle_min + 0.01f, throttle_max - 0.01f);
            }
        }
    }

    _tecs.update_pitch_throttle(_R_nb, pitch_for_tecs,
                                _global_pos.alt, alt_sp,
                                airspeed_sp, _airspeed, _eas2tas,
                                climbout_mode, climbout_pitch_min_rad,
                                throttle_min, throttle_max, throttle_cruise,
                                pitch_min_rad, pitch_max_rad);

    tecs_status_publish();
}

FixedwingPositionControl *FixedwingPositionControl::instantiate(int argc, char *argv[]) {
    return new FixedwingPositionControl();
}

int FixedwingPositionControl::task_spawn(int argc, char *argv[]) {
    _task_id = px4_task_spawn_cmd("fw_pos_control_l1",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_POSITION_CONTROL,
                                  1810,
                                  (px4_main_t) &run_trampoline,
                                  (char *const *) argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

int FixedwingPositionControl::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command");
}

int FixedwingPositionControl::print_usage(const char *reason) {
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
fw_pos_control_l1 is the fixed wing position controller.
)DESCR_STR");


    PRINT_MODULE_USAGE_NAME("fw_pos_control_l1", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int FixedwingPositionControl::print_status() {
    PX4_INFO("Running");

    return 0;
}

int fw_pos_control_l1_main(int argc, char *argv[]) {
    return FixedwingPositionControl::main(argc, argv);
}
