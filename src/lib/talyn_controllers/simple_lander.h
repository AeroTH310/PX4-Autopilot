/**
 * @file simple_lander.h
 * Simple controller to stabilize vtol aircraft and then hold a constant
 * downward velocity.
 *
 * @author Thomas Hamilton <thomas@zenithflight.com>
 */

#pragma once

#include <cmath>

class SimpleLander
{
public:
	SimpleLander();
	~SimpleLander();

	void set_pause(float pause_time){ _pause = pause_time; };
	float get_pause(){ return _pause; };

	void set_land_speed(float speed){ _land_speed_mps = speed; };
	float get_land_speed(){ return _land_speed_mps; };

	void set_hover_throttle(float throttle){ _hover_throttle = throttle; };
	float get_hover_throttle(){ return _hover_throttle; };

	void set_minimum_throttle(float throttle){ _minimum_throttle = throttle; };
	float get_minimum_throttle(){ return _minimum_throttle; };

	struct simple_lander_states
	{
		bool engage_lander;
		float velocity[3];
		float attitude[4];
	};
	void set_states(simple_lander_states input_states){ _lander_states = input_states; };

	struct simple_lander_controls
	{
		float velocity[3];
		float attitude[4];
		float thrust;
	};
	simple_lander_controls get_control(){ return _lander_controls; };

	struct simple_lander_gains
	{
		float proprtnl[3];
	};
	void set_gains(simple_lander_gains input_gains){ _lander_gains = input_gains; };
	simple_lander_gains get_gains(){ return _lander_gains; };

	void update_controller(float time_step);
	void detilt_initial_attitude();
	void compute_control_tilt_thrust();

private:
	float _pause{0.0f};
	float _land_speed_mps{1.0f};
	bool _lander_engaged{false};
	float _lander_time{0.0f};
	//   The vehicle attitude when land mode engaged
	float _initial_attitude[4] = {1.0f, 0.0f, 0.0f};
	//   The de-tilted initial vehicle attitude
	float _detilt_init_attd[4] = {1.0f, 0.0f, 0.0f};

	float _vel_err_wld[3] = {0.0f, 0.0f, 0.0f};

	float _vel_err_bdy[3] = {0.0f, 0.0f, 0.0f};

	float _acc_cmd_bdy[3] = {0.0f, 0.0f, 0.0f};

	float _acc_cmd_wld[3] = {0.0f, 0.0f, 0.0f};

	float _thrust[3] = {0.0f, 0.0f, 0.0f};
	float _hover_throttle = 0.5f;
	float _minimum_throttle = 0.1f;
	float _control_tilt[4] = {1.0f, 0.0f, 0.0f, 0.0f};

	simple_lander_states _lander_states {.engage_lander = false,
		.velocity = {0.0f, 0.0f, 0.0f,}, .attitude = {1.0f, 0.0f, 0.0f, 0.0f}};

	simple_lander_controls _lander_controls {.velocity = {0.0f, 0.0f, 0.0f},
		.attitude = {1.0f, 0.0f, 0.0f, 0.0f,}, .thrust = 0.0f};

	simple_lander_gains _lander_gains {.proprtnl = {0.1f, 0.1f, 0.1f}};
};
