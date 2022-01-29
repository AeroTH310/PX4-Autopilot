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

	//   Helper structures and functions to control parameters.
	void set_pause(float pause_time){ _pause = pause_time; };
	float get_pause(){ return _pause; };

	void set_land_speed(float speed){ _land_speed_mps = speed; };
	float get_land_speed(){ return _land_speed_mps; };

	void set_hover_throttle(float throttle){ _hover_throttle = throttle; };
	float get_hover_throttle(){ return _hover_throttle; };

	void set_minimum_throttle(float throttle){ _minimum_throttle = throttle; };
	float get_minimum_throttle(){ return _minimum_throttle; };

	struct simple_lander_gains
	{
		float proprtnl[3];
	};
	void set_gains(simple_lander_gains input_gains){ _lander_gains = input_gains; };
	simple_lander_gains get_gains(){ return _lander_gains; };

	//   Helper structure and function to provide control feedback.  This member
	// should be called:
	//  - With updated engage_lander state if it has changed since the last
	//      controller update
	//  - With updated velocity if the vehicle's velocity has changed since the last
	//      controller update
	//  - With updated attitude if the engage_lander state has transitioned from
	//      false to true since the last controller update
	// before the next controller update
	struct simple_lander_states
	{
		bool engage_lander;
		float velocity[3];
		float attitude[4];
	};
	void set_states(simple_lander_states input_states){ _lander_states = input_states; };

	//   Helper structure and function to read the lander control solution.  This
	// member should be called any time the control solution is needed, but it will
	// not change until the next controller update.
	struct simple_lander_controls
	{
		float velocity[3];
		float attitude[4];
		float thrust;
	};
	simple_lander_controls get_control(){ return _lander_controls; };

	//   This member updates the controller state by the time step provided.  It
	// should be called at a frequency of at least 10Hz (the time step should never
	// be larger than 0.1s).  The controller can run at lower rates, but this
	// frequency is required to control most users' aircraft.
	void update_controller(float time_step);

private:
	//   Helper for computing nominal attitude solution
	void detilt_initial_attitude();
	//   Helper for computing tilt for landing control
	void compute_control_tilt_thrust();

	//   Parameter memory
	float _pause{0.0f};
	float _land_speed_mps{1.0f};
	float _hover_throttle = 0.5f;
	float _minimum_throttle = 0.1f;
	simple_lander_gains _lander_gains {.proprtnl = {0.1f, 0.1f, 0.1f}};

	//   Engaged state memory
	bool _lander_engaged{false};
	//   Engaged time memory
	float _lander_time{0.0f};

	//   Vehicle attitude when land mode engaged
	float _initial_attitude[4] = {1.0f, 0.0f, 0.0f};
	//   De-tilted initial vehicle attitude
	float _detilt_init_attd[4] = {1.0f, 0.0f, 0.0f};
	//   Vehicle attitude tilt for control
	float _control_tilt[4] = {1.0f, 0.0f, 0.0f, 0.0f};

	//   Internal states
	simple_lander_states _lander_states {.engage_lander = false,
		.velocity = {0.0f, 0.0f, 0.0f,}, .attitude = {1.0f, 0.0f, 0.0f, 0.0f}};
	//   Internal controls solution
	simple_lander_controls _lander_controls {.velocity = {0.0f, 0.0f, 0.0f},
		.attitude = {1.0f, 0.0f, 0.0f, 0.0f,}, .thrust = 0.0f};
};
