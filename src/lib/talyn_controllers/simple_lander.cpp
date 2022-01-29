/**
 * @file simple_lander.cpp
 * Simple controller to stabilize vtol aircraft and then hold a constant
 * downward velocity.
 *
 * @author Thomas Hamilton <thomas@zenithflight.com>
 */

#include "simple_lander.h"

SimpleLander::SimpleLander()
{
}

SimpleLander::~SimpleLander()
{
}

void SimpleLander::update_controller(float time_step)
{
	//   Detect "engage lander" signal edge, set engage state, reset time, and save
	// and process attitude when lander is engaged.
	if (_lander_states.engage_lander && !_lander_engaged) {

		_lander_engaged = true;

		_lander_time = 0.0f;

		_initial_attitude[0] = _lander_states.attitude[0];
		_initial_attitude[1] = _lander_states.attitude[1];
		_initial_attitude[2] = _lander_states.attitude[2];
		_initial_attitude[3] = _lander_states.attitude[3];

		//   De-tilt the initial attitude through the smallest angle which aligns the
		// body and world z axes.  This is the presumed desired attitude if no control
		// input is required to maintain landing zone.
		detilt_initial_attitude();

	//   Lander will compute signals to hold aircraft still if it is pausing.
	// Lander will maintain a constant downward velocity with a magnitude of
	// _land_speed_mps if no longer pausing.
	} else if (_lander_states.engage_lander) {

		_lander_time += time_step;

		if (_lander_time < _pause) {

			_lander_controls.velocity[0] = 0.0f;
			_lander_controls.velocity[1] = 0.0f;
			_lander_controls.velocity[2] = 0.0f;
		} else {

			_lander_controls.velocity[0] = 0.0f;
			_lander_controls.velocity[1] = 0.0f;
			_lander_controls.velocity[2] = _land_speed_mps;
		}

	//   Lander will compute signals to hold aircraft still if it is disengaged.  It
	// is up to the user to use (or not use) this controller output correctly.
	} else if (!_lander_states.engage_lander) {

		_lander_engaged = false;

		_lander_controls.velocity[0] = 0.0f;
		_lander_controls.velocity[1] = 0.0f;
		_lander_controls.velocity[2] = 0.0f;
	}

	//   Compute an acceleration command (which will remain implicit) from the
	// velocity error, normalize, feedforward a throttle value for hover, and split
	// into magnitude and direction parts.
	compute_control_tilt_thrust();

	//   Now tilt the detilted initial attitude to obtain the desired attitude for
	// control purposes by the Hamilton product.
	_lander_controls.attitude[0] = _control_tilt[0]*_detilt_init_attd[0];
	_lander_controls.attitude[1] = _control_tilt[1]*_detilt_init_attd[0]
		+ _control_tilt[2]*_detilt_init_attd[3];
	_lander_controls.attitude[2] = _control_tilt[2]*_detilt_init_attd[0]
		- _control_tilt[1]*_detilt_init_attd[3];
	_lander_controls.attitude[3] = _control_tilt[0]*_detilt_init_attd[3];
}

void SimpleLander::detilt_initial_attitude()
{
	// TODO:  Need to handle corner case where body z axis is very close to opposite
	// of world z axis (vehicle inverted).
	const float _z_hypot = sqrtf(_initial_attitude[0]*_initial_attitude[0]
		+ _initial_attitude[3]*_initial_attitude[3]);
	_detilt_init_attd[0] = _initial_attitude[0]/_z_hypot;
	_detilt_init_attd[1] = 0.0f;
	_detilt_init_attd[2] = 0.0f;
	_detilt_init_attd[3] = _initial_attitude[3]/_z_hypot;
}

void SimpleLander::compute_control_tilt_thrust()
{
	//   Compute the velocity error in world frame.
	const float _vel_err_wld[3] = {
		_lander_states.velocity[0] - _lander_controls.velocity[0],
		_lander_states.velocity[1] - _lander_controls.velocity[1],
		_lander_states.velocity[2] - _lander_controls.velocity[2]
	};

	//   Convert velocity error to body frame using the (current) attitude DCM.
	const float q0q0 = _lander_states.attitude[0]*_lander_states.attitude[0];
	const float q0q1 = _lander_states.attitude[0]*_lander_states.attitude[1];
	const float q0q2 = _lander_states.attitude[0]*_lander_states.attitude[2];
	const float q0q3 = _lander_states.attitude[0]*_lander_states.attitude[3];
	const float q1q1 = _lander_states.attitude[1]*_lander_states.attitude[1];
	const float q1q2 = _lander_states.attitude[1]*_lander_states.attitude[2];
	const float q1q3 = _lander_states.attitude[1]*_lander_states.attitude[3];
	const float q2q2 = _lander_states.attitude[2]*_lander_states.attitude[2];
	const float q2q3 = _lander_states.attitude[2]*_lander_states.attitude[3];
	const float q3q3 = _lander_states.attitude[3]*_lander_states.attitude[3];
	const float dcm00 = q0q0 + q1q1 - q2q2 - q3q3;
	const float dcm01 = 2*(q1q2 - q0q3);
	const float dcm02 = 2*(q0q2 + q1q3);
	const float dcm10 = 2*(q0q3 + q1q2);
	const float dcm11 = q0q0 - q1q1 + q2q2 - q3q3;
	const float dcm12 = 2*(q2q3 - q0q1);
	const float dcm20 = 2*(q1q3 - q0q2);
	const float dcm21 = 2*(q0q1 + q2q3);
	const float dcm22 = q0q0 - q1q1 - q2q2 + q3q3;

	const float _vel_err_bdy[3] = {
		dcm00*_vel_err_wld[0] + dcm10*_vel_err_wld[1] + dcm20*_vel_err_wld[2],
		dcm01*_vel_err_wld[0] + dcm11*_vel_err_wld[1] + dcm21*_vel_err_wld[2],
		dcm02*_vel_err_wld[0] + dcm12*_vel_err_wld[1] + dcm22*_vel_err_wld[2]
	};

	//   Apply the gains (which are in the body frame) to the velocity error.  Note
	// that the acceleration command is pointing in the direction of increasing the
	// error (it would be positive feedback).
	const float _acc_cmd_bdy[3] = {
		_lander_gains.proprtnl[0]*_vel_err_bdy[0],
		_lander_gains.proprtnl[1]*_vel_err_bdy[1],
		_lander_gains.proprtnl[2]*_vel_err_bdy[2]
	};

	//   Rotate back to world frame
	const float _acc_cmd_wld[3] = {
		dcm00*_acc_cmd_bdy[0] + dcm01*_acc_cmd_bdy[1] + dcm02*_acc_cmd_bdy[2],
		dcm10*_acc_cmd_bdy[0] + dcm11*_acc_cmd_bdy[1] + dcm12*_acc_cmd_bdy[2],
		dcm20*_acc_cmd_bdy[0] + dcm21*_acc_cmd_bdy[1] + dcm22*_acc_cmd_bdy[2]
	};

	//   Negate acceleration command so that feedback is negative, scale to
	// propulsion system throttle signal, subtract offset for hover throttle.  We're
	// going to assume that the vehicle isn't capable of suddenly inverting to
	// increase the descent rate if the z component of the thrust command goes
	// positive.  Therefore the z component will be upper-limited to the negated
	// minimum throttle.  Non-zero minimum throttle also helps prevent rapid
	// attitude command fluctuations when z thrust is close to zero (a thrust
	// command with only x and y components would result in an attitude command with
	// the body z axis parallel to the horizon).
	float _thrust[3] = {
		-_acc_cmd_wld[0],
		-_acc_cmd_wld[1],
		-_acc_cmd_wld[2] - _hover_throttle
	};
	_thrust[2] = _thrust[2] < -_minimum_throttle ? _thrust[2] : -_minimum_throttle;

	//   Compute the total thrust value
	_lander_controls.thrust = sqrt(_thrust[0]*_thrust[0]
		+ _thrust[1]*_thrust[1] + _thrust[2]*_thrust[2]);

	//   Compute the minimum attitude quaternion which would point the body z axis
	// to the polar opposite of the desired thrust vector.  Convert the thrust
	// vector to a unit vector
	_thrust[0] = _thrust[0]/_lander_controls.thrust;
	_thrust[1] = _thrust[1]/_lander_controls.thrust;
	_thrust[2] = _thrust[2]/_lander_controls.thrust;
	//   Compute the quaternion which rotates the world z axis antipodal to the
	// world thrust vector.
	_control_tilt[0] = sqrt(0.5f - 0.5f*_thrust[2]);
	_control_tilt[1] = 0.5f*_thrust[1]/_control_tilt[0];
	_control_tilt[2] = -0.5f*_thrust[0]/_control_tilt[0];
	_control_tilt[3] = 0.0f;
}
