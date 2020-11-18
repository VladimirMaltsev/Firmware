/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file mavlink_receiver.h
 * MAVLink receiver thread
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#pragma once

#include <perf/perf_counter.h>
#include <uORB/uORB.h>

#include <uORB/topics/airspeed.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/ping.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/stg_status.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/engine_status.h>

#include "mavlink_ftp.h"
#include "mavlink_log_handler.h"
#include "mavlink_mission.h"
#include "mavlink_parameters.h"
#include "mavlink_timesync.h"

static float NTC_temp[191] = {3.270647047, 3.268661408, 3.266557847, 3.264330493, 3.261973262, 3.259479846, 3.256843718, 3.254058126, 3.251116092, 3.248010413, 3.244733658, 3.24127817, 3.237636065, 3.233799239, 3.229759364, 3.225507894, 3.221036072, 3.216334932, 3.211395306, 3.206207834, 3.200762969, 3.195050992, 3.189062018, 3.182786011, 3.176212802, 3.169332099, 3.162133505, 3.154606541, 3.146740664, 3.138525285, 3.129949802, 3.121003617, 3.111676167, 3.101956952, 3.091835567, 3.081301733, 3.070345329, 3.058956433, 3.047125353, 3.034842666, 3.022099264, 3.008886385, 2.995195661, 2.981019161, 2.966349431, 2.951179537, 2.935503114, 2.919314405, 2.902608306, 2.885380412, 2.867627054, 2.849345343, 2.830533212, 2.811189449, 2.791313738, 2.77090669, 2.749969871, 2.728505837, 2.706518152, 2.684011413, 2.660991263, 2.637464409, 2.613438626, 2.588922762, 2.563926737, 2.538461538, 2.512539209, 2.486172828, 2.459376496, 2.432165302, 2.404555294, 2.376563447, 2.348207613, 2.319506482, 2.290479529, 2.261146959, 2.231529651, 2.201649092, 2.171527317, 2.14118684, 2.110650581, 2.079941799, 2.049084019, 2.018100956, 1.987016443, 1.955854358, 1.924638548, 1.893392761, 1.862140575, 1.830905327, 1.79971005, 1.768577408, 1.737529635, 1.706588483, 1.675775161, 1.645110292, 1.614613865, 1.584305191, 1.554202873, 1.524324763, 1.494687945, 1.4653087, 1.436202493, 1.407383955, 1.378866871, 1.350664173, 1.322787935, 1.295249373, 1.268058851, 1.241225883, 1.214759144, 1.188666485, 1.162954943, 1.137630763, 1.112699412, 1.088165607, 1.06403333, 1.04030586, 1.016985792, 0.9940750665, 0.9715749975, 0.9494862984, 0.9278091115, 0.9065430362, 0.8856871577, 0.8652400757, 0.845199933, 0.8255644433, 0.8063309197, 0.7874963012, 0.7690571801, 0.7510098277, 0.7333502198, 0.7160740612, 0.6991768097, 0.6826536987, 0.6664997595, 0.6507098423, 0.6352786363, 0.6202006893, 0.6054704258, 0.5910821644, 0.5770301345, 0.5633084916, 0.5499113325, 0.5368327083, 0.5240666383, 0.5116071214, 0.4994481477, 0.4875837092, 0.4760078092, 0.4647144716, 0.4536977493, 0.4429517316, 0.4324705517, 0.4222483926, 0.4122794933, 0.4025581543, 0.3930787418, 0.3838356925, 0.3748235171, 0.3660368041, 0.3574702224, 0.3491185239, 0.3409765462, 0.3330392137, 0.3253015401, 0.317758629, 0.3104056751, 0.303237965, 0.2962508779, 0.2894398856, 0.2828005528, 0.2763285369, 0.2700195879, 0.263869548, 0.2578743509, 0.2520300213, 0.2463326742, 0.2407785136, 0.2353638322, 0.2300850098, 0.2249385123, 0.2199208907, 0.2150287796, 0.2102588961, 0.2056080384, 0.2010730842, 0.1966509897, 0.1923387879, 0.1881335873};
static uint8_t fuel_level{100};

class Mavlink;

class MavlinkReceiver
{
public:
	/**
	 * Constructor
	 */
	MavlinkReceiver(Mavlink *parent);

	/**
	 * Destructor, also kills the mavlinks task.
	 */
	~MavlinkReceiver();

	/**
	 * Display the mavlink status.
	 */
	void print_status();

	/**
	 * Start the receiver thread
	 */
	static void receive_start(pthread_t *thread, Mavlink *parent);

	static void *start_helper(void *context);

private:
	int lastOverride = 0;
	bool hasOverrides = false;
	bool remoteMode = false;

	void acknowledge(uint8_t sysid, uint8_t compid, uint16_t command, uint8_t result);

	/**
	 * Common method to handle both mavlink command types. T is one of mavlink_command_int_t or mavlink_command_long_t.
	 */
	template<class T>
	void handle_message_command_both(mavlink_message_t *msg, const T &cmd_mavlink,
					 const vehicle_command_s &vehicle_command);

	uint8_t handle_request_message_command(uint16_t message_id, float param2 = 0.0f, float param3 = 0.0f,
					float param4 = 0.0f,
					float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f);

	void handle_message(mavlink_message_t *msg);
	void handle_message_adsb_vehicle(mavlink_message_t *msg);
	void handle_message_att_pos_mocap(mavlink_message_t *msg);
	void handle_message_battery_status(mavlink_message_t *msg);
	void handle_message_collision(mavlink_message_t *msg);
	void handle_message_command_ack(mavlink_message_t *msg);
	void handle_message_command_int(mavlink_message_t *msg);
	void handle_message_command_long(mavlink_message_t *msg);
	void handle_message_debug(mavlink_message_t *msg);
	void handle_message_debug_float_array(mavlink_message_t *msg);
	void handle_message_debug_vect(mavlink_message_t *msg);
	void handle_message_distance_sensor(mavlink_message_t *msg);
	void handle_message_follow_target(mavlink_message_t *msg);
	void handle_message_gps_global_origin(mavlink_message_t *msg);
	void handle_message_gps_rtcm_data(mavlink_message_t *msg);
	void handle_message_heartbeat(mavlink_message_t *msg);
	void handle_message_hil_gps(mavlink_message_t *msg);
	void handle_message_hil_optical_flow(mavlink_message_t *msg);
	void handle_message_hil_sensor(mavlink_message_t *msg);
	void handle_message_hil_state_quaternion(mavlink_message_t *msg);
	void handle_message_landing_target(mavlink_message_t *msg);
	void handle_message_logging_ack(mavlink_message_t *msg);
	void handle_message_manual_control(mavlink_message_t *msg);
	void handle_message_named_value_float(mavlink_message_t *msg);
	void handle_message_obstacle_distance(mavlink_message_t *msg);
	void handle_message_odometry(mavlink_message_t *msg);
	void handle_message_optical_flow_rad(mavlink_message_t *msg);
	void handle_message_ping(mavlink_message_t *msg);
	void handle_message_play_tune(mavlink_message_t *msg);
	void handle_message_radio_status(mavlink_message_t *msg);
	void handle_message_rc_channels_override(mavlink_message_t *msg);
	void handle_message_serial_control(mavlink_message_t *msg);
	void handle_message_set_actuator_control_target(mavlink_message_t *msg);
	void handle_message_set_attitude_target(mavlink_message_t *msg);
	void handle_message_set_mode(mavlink_message_t *msg);
	void handle_message_set_position_target_local_ned(mavlink_message_t *msg);
	void handle_message_trajectory_representation_waypoints(mavlink_message_t *msg);
	void handle_message_vision_position_estimate(mavlink_message_t *msg);
	void handle_message_stg_status_msg(mavlink_message_t *msg);

	void *receive_thread(void *arg);

	/**
	 * Set the interval at which the given message stream is published.
	 * The rate is the number of messages per second.
	 *
	 * @param msgId The ID of the message interval to be set.
	 * @param interval The interval in usec to send the message.
	 * @param data_rate The total link data rate in bytes per second.
	 *
	 * @return PX4_OK on success, PX4_ERROR on fail.
	 */
	int set_message_interval(int msgId, float interval, int data_rate = -1);
	void get_message_interval(int msgId);

	/**
	 * Decode a switch position from a bitfield.
	 */
	switch_pos_t decode_switch_pos(uint16_t buttons, unsigned sw);

	/**
	 * Decode a switch position from a bitfield and state.
	 */
	int decode_switch_pos_n(uint16_t buttons, unsigned sw);

	bool evaluate_target_ok(int command, int target_system, int target_component);

	void send_flight_information();

	void send_storage_information(int storage_id);

	void send_manual_overrides (uint16_t values[]);

	Mavlink	*_mavlink;

	MavlinkFTP			_mavlink_ftp;
	MavlinkLogHandler		_mavlink_log_handler;
	MavlinkTimesync			_mavlink_timesync;
	MavlinkMissionManager		_mission_manager;
	MavlinkParametersManager	_parameters_manager;

	mavlink_status_t _status{}; ///< receiver status, used for mavlink_parse_char()

	map_projection_reference_s _hil_local_proj_ref {};
	offboard_control_mode_s _offboard_control_mode{};

	vehicle_attitude_s _att {};
	vehicle_local_position_s _hil_local_pos {};
	vehicle_land_detected_s _hil_land_detector {};
	vehicle_control_mode_s _control_mode {};

	orb_advert_t _accel_pub{nullptr};
	orb_advert_t _actuator_controls_pubs[4] {nullptr, nullptr, nullptr, nullptr};
	orb_advert_t _airspeed_pub{nullptr};
	orb_advert_t _att_sp_pub{nullptr};
	orb_advert_t _attitude_pub{nullptr};
	orb_advert_t _baro_pub{nullptr};
	orb_advert_t _battery_pub{nullptr};
	orb_advert_t _cmd_pub{nullptr};
	orb_advert_t _collision_report_pub{nullptr};
	orb_advert_t _command_ack_pub{nullptr};
	orb_advert_t _debug_array_pub{nullptr};
	orb_advert_t _debug_key_value_pub{nullptr};
	orb_advert_t _debug_value_pub{nullptr};
	orb_advert_t _debug_vect_pub{nullptr};
	orb_advert_t _distance_sensor_pub{nullptr};
	orb_advert_t _flow_distance_sensor_pub{nullptr};
	orb_advert_t _flow_pub{nullptr};
	orb_advert_t _follow_target_pub{nullptr};
	orb_advert_t _global_pos_pub{nullptr};
	orb_advert_t _gps_inject_data_pub{nullptr};
	orb_advert_t _gps_pub{nullptr};
	orb_advert_t _gyro_pub{nullptr};
	orb_advert_t _hil_distance_sensor_pub{nullptr};
	orb_advert_t _land_detector_pub{nullptr};
	orb_advert_t _landing_target_pose_pub{nullptr};
	orb_advert_t _local_pos_pub{nullptr};
	orb_advert_t _mag_pub{nullptr};
	orb_advert_t _manual_pub{nullptr};
	orb_advert_t _mocap_odometry_pub{nullptr};
	orb_advert_t _obstacle_distance_pub{nullptr};
	orb_advert_t _offboard_control_mode_pub{nullptr};
	orb_advert_t _ping_pub{nullptr};
	orb_advert_t _pos_sp_triplet_pub{nullptr};
	orb_advert_t _radio_status_pub{nullptr};
	orb_advert_t _rates_sp_pub{nullptr};
	orb_advert_t _rc_pub{nullptr};
	orb_advert_t _trajectory_waypoint_pub{nullptr};
	orb_advert_t _transponder_report_pub{nullptr};
	orb_advert_t _visual_odometry_pub{nullptr};
	orb_advert_t _stg_status_msg_pub{nullptr};
	orb_advert_t	_mavlink_log_pub{nullptr};

	orb_advert_t    act_pub{nullptr};
    	orb_advert_t    act_pub0{nullptr};
    	orb_advert_t    act_pub1{nullptr};
    	orb_advert_t    act_pub2{nullptr};
    	orb_advert_t    act_pub3{nullptr};

	struct actuator_controls_s act = {};
    	struct actuator_controls_s act0 = {};
    	struct actuator_controls_s act1 = {};
    	struct actuator_controls_s act2 = {};
    	struct actuator_controls_s act3 = {};

	static constexpr int _gps_inject_data_queue_size{6};

	int _actuator_armed_sub{orb_subscribe(ORB_ID(actuator_armed))};
	int _control_mode_sub{orb_subscribe(ORB_ID(vehicle_control_mode))};
	int _vehicle_attitude_sub{orb_subscribe(ORB_ID(vehicle_attitude))};
	int _adc_report_sub{orb_subscribe(ORB_ID(adc_report))};

	int _orb_class_instance{-1};

	uint64_t _global_ref_timestamp{0};

	bool _hil_local_proj_inited{false};

	float _hil_local_alt0{0.0f};

	static constexpr unsigned MOM_SWITCH_COUNT{8};

	uint8_t _mom_switch_pos[MOM_SWITCH_COUNT] {};
	uint16_t _mom_switch_state{0};

	param_t _p_bat_emergen_thr{PARAM_INVALID};
	param_t _p_bat_crit_thr{PARAM_INVALID};
	param_t _p_bat_low_thr{PARAM_INVALID};
	param_t _p_flow_rot{PARAM_INVALID};
	param_t _p_flow_maxr{PARAM_INVALID};
	param_t _p_flow_minhgt{PARAM_INVALID};
	param_t _p_flow_maxhgt{PARAM_INVALID};



	// Disallow copy construction and move assignment.
	MavlinkReceiver(const MavlinkReceiver &) = delete;
	MavlinkReceiver operator=(const MavlinkReceiver &) = delete;
};
