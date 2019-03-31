
/**
 * @file mc_pos_control_main.cpp
 * @author jiangyuanqing2008@163.com
 * @date 2019/03/26
 */

#ifndef _MC_POS_CONTROL_MAIN_
#define _MC_POS_CONTROL_MAIN_


#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module_params.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>

#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <float.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include <controllib/blocks.hpp>

#include <lib/FlightTasks/FlightTasks.hpp>
#include "PositionControl.hpp"
#include "Utility/ControlMath.hpp"


/* 
 * 告知编译器按照C++语言的格式引用C语言的main函数
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);


class MulticopterPositionControl : public control::SuperBlock, public ModuleParams
{
	public:
		/**
		 * Constructor
		 */
		MulticopterPositionControl();

		/**
		 * Destructor, also kills task.
		 */
		~MulticopterPositionControl();

		/**
		 * Start task.
		 *
		 * @return		OK on success.
		 */
		int		start();

		bool		cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r,
						  const matrix::Vector3f &line_a, const matrix::Vector3f &line_b, matrix::Vector3f &res);

	private:

		/** Time in us that direction change condition has to be true for direction change state */
		static constexpr uint64_t DIRECTION_CHANGE_TRIGGER_TIME_US = 100000;

		bool		_task_should_exit = false;			/**<true if task should exit */
		bool		_gear_state_initialized = false;		/**<true if the gear state has been initialized */
		bool 		_reset_pos_sp = true;  				/**<true if position setpoint needs a reset */
		bool 		_reset_alt_sp = true; 				/**<true if altitude setpoint needs a reset */
		bool 		_do_reset_alt_pos_flag = true; 		/**< TODO: check if we need this */
		bool		_mode_auto = false ;  				/**<true if in auot mode */
		bool 		_pos_hold_engaged = false; 			/**<true if hold positon in xy desired */
		bool 		_alt_hold_engaged = false; 			/**<true if hold in z desired */
		bool 		_run_pos_control = true;  			/**< true if position controller should be used */
		bool 		_run_alt_control = true; 			/**<true if altitude controller should be used */
		bool 		_reset_int_z = true; 				/**<true if reset integral in z */
		bool 		_reset_int_xy = true; 				/**<true if reset integral in xy */
		bool		 _reset_yaw_sp = true; 				/**<true if reset yaw setpoint */
		bool 		_hold_offboard_xy = false; 			/**<TODO : check if we need this extra hold_offboard flag */
		bool 		_hold_offboard_z = false;
		bool 		_in_smooth_takeoff = false; 				/**<true if takeoff ramp is applied */
		bool 		_in_landing = false;				/**<true if landing descent (only used in auto) */
		bool 		_lnd_reached_ground = false; 		/**<true if controller assumes the vehicle has reached the ground after landing */
		bool 		_triplet_lat_lon_finite = true; 		/**<true if triplets current is non-finite */
		bool		_terrain_follow = false;			/**<true is the position controller is controlling height above ground */

		int		_control_task;			/**< task handle for task */
		orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

		int		_vehicle_status_sub;		/**< vehicle status subscription */
		int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
		int		_vehicle_attitude_sub;		/**< control state subscription */
		int		_control_mode_sub;		/**< vehicle control mode subscription */
		int		_params_sub;			/**< notification of parameter updates */
		int		_manual_sub;			/**< notification of manual control updates */
		int		_local_pos_sub;			/**< vehicle local position */
		int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
		int		_home_pos_sub; 			/**< home position */

		orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
		orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */

		orb_id_t _attitude_setpoint_id;

		struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
		struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
		struct vehicle_attitude_s			_att;			/**< vehicle attitude */
		struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
		struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
		struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
		struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
		struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
		struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
		struct home_position_s				_home_pos; 				/**< home position */

		DEFINE_PARAMETERS(
			(ParamInt<px4::params::MPC_FLT_TSK>) _test_flight_tasks, /**< temporary flag for the transition to flight tasks */
			(ParamFloat<px4::params::MPC_MANTHR_MIN>) _manual_thr_min, /**< minimal throttle output when flying in manual mode */
			(ParamFloat<px4::params::MPC_MANTHR_MAX>) _manual_thr_max, /**< maximal throttle output when flying in manual mode */
			(ParamFloat<px4::params::MPC_XY_MAN_EXPO>)
			_xy_vel_man_expo, /**< ratio of exponential curve for stick input in xy direction pos mode */
			(ParamFloat<px4::params::MPC_Z_MAN_EXPO>)
			_z_vel_man_expo, /**< ratio of exponential curve for stick input in xy direction pos mode */
			(ParamFloat<px4::params::MPC_HOLD_DZ>)
			_hold_dz, /**< deadzone around the center for the sticks when flying in position mode */
			(ParamFloat<px4::params::MPC_ACC_HOR_MAX>)
			_acceleration_hor_max, /**<maximum velocity setpoint slewrate for auto & fast manual brake */
			(ParamFloat<px4::params::MPC_ACC_HOR>)
			_acceleration_hor, /**<acceleration for auto and maximum for manual in velocity control mode*/
			(ParamFloat<px4::params::MPC_DEC_HOR_SLOW>)
			_deceleration_hor_slow, /**< slow velocity setpoint slewrate for manual deceleration*/
			(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _acceleration_z_max_up, /** max acceleration up */
			(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _acceleration_z_max_down, /** max acceleration down */
			(ParamFloat<px4::params::MPC_CRUISE_90>)
			_cruise_speed_90, /**<speed when angle is 90 degrees between prev-current/current-next*/
			(ParamFloat<px4::params::MPC_VEL_MANUAL>)
			_velocity_hor_manual, /**< target velocity in manual controlled mode at full speed*/
			(ParamFloat<px4::params::NAV_ACC_RAD>)
			_nav_rad, /**< radius that is used by navigator that defines when to update triplets */
			(ParamFloat<px4::params::MPC_TKO_RAMP_T>) _takeoff_ramp_time, /**< time contant for smooth takeoff ramp */
			(ParamFloat<px4::params::MPC_JERK_MAX>)
			_jerk_hor_max, /**< maximum jerk in manual controlled mode when braking to zero */
			(ParamFloat<px4::params::MPC_JERK_MIN>)
			_jerk_hor_min, /**< minimum jerk in manual controlled mode when braking to zero */
			(ParamFloat<px4::params::MIS_YAW_ERR>)
			_mis_yaw_error, /**< yaw error threshold that is used in mission as update criteria */

			(ParamFloat<px4::params::MPC_THR_MIN>) _thr_min,
			(ParamFloat<px4::params::MPC_THR_MAX>) _thr_max,
			(ParamFloat<px4::params::MPC_THR_HOVER>) _thr_hover,
			(ParamFloat<px4::params::MPC_Z_P>) _z_p,
			(ParamFloat<px4::params::MPC_Z_VEL_P>) _z_vel_p,
			(ParamFloat<px4::params::MPC_Z_VEL_I>) _z_vel_i,
			(ParamFloat<px4::params::MPC_Z_VEL_D>) _z_vel_d,
			(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _vel_max_up,
			(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _vel_max_down,
			(ParamFloat<px4::params::MPC_LAND_ALT1>) _slow_land_alt1,
			(ParamFloat<px4::params::MPC_LAND_ALT2>) _slow_land_alt2,
			(ParamFloat<px4::params::MPC_XY_P>) _xy_p,
			(ParamFloat<px4::params::MPC_XY_VEL_P>) _xy_vel_p,
			(ParamFloat<px4::params::MPC_XY_VEL_I>) _xy_vel_i,
			(ParamFloat<px4::params::MPC_XY_VEL_D>) _xy_vel_d,
			(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _vel_max_xy_param,
			(ParamFloat<px4::params::MPC_XY_CRUISE>) _vel_cruise_xy,
			(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _tilt_max_air_deg,
			(ParamFloat<px4::params::MPC_LAND_SPEED>) _land_speed,
			(ParamFloat<px4::params::MPC_TKO_SPEED>) _tko_speed,
			(ParamFloat<px4::params::MPC_TILTMAX_LND>) _tilt_max_land_deg,
			(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _man_tilt_max_deg,
			(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _man_yaw_max_deg,
			(ParamFloat<px4::params::MC_YAWRATE_MAX>) _global_yaw_max_deg,
			(ParamFloat<px4::params::MC_YAW_P>) _mc_att_yaw_p,
			(ParamFloat<px4::params::MPC_HOLD_MAX_XY>) _hold_max_xy,
			(ParamFloat<px4::params::MPC_HOLD_MAX_Z>) _hold_max_z,
			(ParamInt<px4::params::MPC_ALT_MODE>) _alt_mode,
			(ParamFloat<px4::params::RC_FLT_CUTOFF>) _rc_flt_cutoff,
			(ParamFloat<px4::params::RC_FLT_SMP_RATE>) _rc_flt_smp_rate,
			(ParamFloat<px4::params::MPC_ACC_HOR_ESTM>) _acc_max_estimator_xy

		);


		control::BlockDerivative _vel_x_deriv;
		control::BlockDerivative _vel_y_deriv;
		control::BlockDerivative _vel_z_deriv;


		FlightTasks _flight_tasks; /**< class handling all ways to generate position controller setpoints */
		PositionControl _control{}; /**< class handling the core PID position controller */

		systemlib::Hysteresis _manual_direction_change_hysteresis;

		math::LowPassFilter2p _filter_manual_pitch;
		math::LowPassFilter2p _filter_manual_roll;

		enum manual_stick_input {
			brake,
			direction_change,
			acceleration,
			deceleration
		};

		manual_stick_input _user_intention_xy; /**< defines what the user intends to do derived from the stick input */
		manual_stick_input
		_user_intention_z; /**< defines what the user intends to do derived from the stick input in z direciton */

		matrix::Vector3f _pos_p;
		matrix::Vector3f _vel_p;
		matrix::Vector3f _vel_i;
		matrix::Vector3f _vel_d;
		float _tilt_max_air; /**< maximum tilt angle [rad] */
		float _tilt_max_land; /**< maximum tilt angle during landing [rad] */
		float _man_tilt_max;
		float _man_yaw_max;
		float _global_yaw_max;

		struct map_projection_reference_s _ref_pos;
		float _ref_alt;
		bool _ref_alt_is_global; /** true when the reference altitude is defined in a global reference frame */
		hrt_abstime _ref_timestamp;
		hrt_abstime _last_warn;

		matrix::Vector3f _thrust_int;
		matrix::Vector3f _pos;
		matrix::Vector3f _pos_sp;
		matrix::Vector3f _vel;
		matrix::Vector3f _vel_sp;
		matrix::Vector3f _vel_prev;			/**< velocity on previous step */
		matrix::Vector3f _vel_sp_prev;
		matrix::Vector3f _vel_err_d;		/**< derivative of current velocity */
		matrix::Vector3f _curr_pos_sp;  /**< current setpoint of the triplets */
		matrix::Vector3f _prev_pos_sp; /**< previous setpoint of the triples */
		matrix::Vector2f _stick_input_xy_prev; /**< for manual controlled mode to detect direction change */

		matrix::Dcmf _R;			/**< rotation matrix from attitude quaternions */
		float _yaw;				/**< yaw angle (euler) */
		float _yaw_takeoff;	/**< home yaw angle present when vehicle was taking off (euler) */
		float _man_yaw_offset; /**< current yaw offset in manual mode */

		float _vel_max_xy;  /**< equal to vel_max except in auto mode when close to target */
		bool _vel_sp_significant; /** true when the velocity setpoint is over 50% of the _vel_max_xy limit */
		float _acceleration_state_dependent_xy; /**< acceleration limit applied in manual mode */
		float _acceleration_state_dependent_z; /**< acceleration limit applied in manual mode in z */
		float _manual_jerk_limit_xy; /**< jerk limit in manual mode dependent on stick input */
		float _manual_jerk_limit_z; /**< jerk limit in manual mode in z */
		float _z_derivative; /**< velocity in z that agrees with position rate */

		float _takeoff_vel_limit; /**< velocity limit value which gets ramped up */

		// counters for reset events on position and velocity states
		// they are used to identify a reset event
		uint8_t _z_reset_counter;
		uint8_t _xy_reset_counter;
		uint8_t _heading_reset_counter;

		matrix::Dcmf _R_setpoint;

		/**
		 * Update our local parameter cache.
		 */
		int		parameters_update(bool force);

		/**
		 * Check for changes in subscribed topics.
		 */
		void		poll_subscriptions();

		float		throttle_curve(float ctl, float ctr);

		/**
		 * Update reference for local position projection
		 */
		void		update_ref();

		/**
		 * Reset position setpoint to current position.
		 *
		 * This reset will only occur if the _reset_pos_sp flag has been set.
		 * The general logic is to first "activate" the flag in the flight
		 * regime where a switch to a position control mode should hold the
		 * very last position. Once switching to a position control mode
		 * the last position is stored once.
		 */
		void		reset_pos_sp();

		/**
		 * Reset altitude setpoint to current altitude.
		 *
		 * This reset will only occur if the _reset_alt_sp flag has been set.
		 * The general logic follows the reset_pos_sp() architecture.
		 */
		void		reset_alt_sp();

		/**
		 * Set position setpoint using manual control
		 */
		void		control_manual();

		void		control_non_manual();

		/**
		 * Set position setpoint using offboard control
		 */
		void		control_offboard();

		/**
		 * Set position setpoint for AUTO
		 */
		void		control_auto();

		void control_position();
		void calculate_velocity_setpoint();
		void calculate_thrust_setpoint();

		void vel_sp_slewrate();

		void update_velocity_derivative();

		void do_control();

		void generate_attitude_setpoint();

		float get_cruising_speed_xy();

		bool in_auto_takeoff();

		float get_vel_close(const matrix::Vector2f &unit_prev_to_current, const matrix::Vector2f &unit_current_to_next);

		void set_manual_acceleration_xy(matrix::Vector2f &stick_input_xy_NED);

		void set_manual_acceleration_z(float &max_acc_z, const float stick_input_z_NED);


		/**
		 * limit altitude based on several conditions
		 */
		void limit_altitude();

		void warn_rate_limited(const char *str);

		bool manual_wants_takeoff();

		bool manual_wants_landing();

		void set_takeoff_velocity(float &vel_sp_z);

		void landdetection_thrust_limit(matrix::Vector3f &thrust_sp);

		void set_idle_state();

		/**
		 * Temporary method for flight control compuation
		 */
		void updateConstraints(Controller::Constraints &constrains);

		void publish_attitude();

		void publish_local_pos_sp();

		/**
		 * Shim for calling task_main from task_create.
		 */
		static int	task_main_trampoline(int argc, char *argv[]);

		/**
		 * Main sensor collection task.
		 */
		void		task_main();
};


#endif