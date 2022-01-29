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
		// body and world z axes.  This is the presumed desred attitude if no control
		// input is required to maintain landing zone; the controller will not attempt to
		// change heading from here.
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
	// control purposes.
	_lander_controls.attitude[0] = _control_tilt[0]*_detilt_init_attd[0];
	_lander_controls.attitude[1] = _control_tilt[1]*_detilt_init_attd[0]
		+ _control_tilt[2]*_detilt_init_attd[3];
	_lander_controls.attitude[2] = _control_tilt[2]*_detilt_init_attd[0]
		- _control_tilt[1]*_detilt_init_attd[3];
	_lander_controls.attitude[3] = _control_tilt[0]*_detilt_init_attd[3];
}

void SimpleLander::detilt_initial_attitude()
{
	//   Find the attitude that results when the initial attitude is rotated by the
	// smallest amount necessary to align the body and world z axes.

	//   The initial attitude is quaternion < q0, q1, q2, q3 >
	// const float q[4] = {_initial_attitude[0], _initial_attitude[1],
	//         _initial_attitude[2], _initial_attitude[3]};

	//   The body z axis in world frame is < z0, z1, z2 >, we can obtain this as the
	// 3rd column of the DCM matrix, which converts a body frame vector to world
	// basis by left multiplication:
	// const float z[3] = {2.0f*(q[0]*q[2] + q[1]*q[3]),
	//         2.0f*(q[2]*q[3] - q[0]*q[1]),
	//         q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]};

	//   The smallest rotation to make the body z axis parallel to world z axis is
	// quaternion < p0, p1, p2, p3 >:
	// p0' = z2 + sqrt((z0^2 + z1^2 + z2^2))
	//     = aa - bb - cc + dd + sqrt(4*(aacc + bbdd + 2*abcd)
	//       + 4*(ccdd + aabb - 2*abcd) + aaaa + bbbb + cccc + dddd
	//       - 2*aabb - 2*aacc + 2*aadd + 2*bbcc - 2*bbdd - 2*ccdd)
	//     = aa - bb - cc + dd + sqrt(4*aabb - 2*aabb + 4*aacc - 2*aacc
	//	 + 4*bbdd - 2*bbdd + 4*ccdd - 2*ccdd + 2*aadd + 2*bbcc
	//       + 8*abcd - 8*abcd + aaaa + bbbb + cccc + dddd)
	//     = aa - bb - cc + dd + sqrt(aaaa + bbbb + cccc + dddd
	//	 + 2*aabb + 2*aacc + 2*aadd + 2*bbcc + 2*bbdd + 2*ccdd
	//     = aa - bb - cc + dd + sqrt((aa + bb + cc + dd)*(aa + bb + cc + dd))
	//     = aa - bb - cc + dd + aa + bb + cc + dd
	//     = 2*aa + 2*dd
	// p1' = z1
	//     = 2*cd - 2*ab
	// p2' = -z0
	//     = -2*ac - 2*bd
	// p3' = 0
	// |p'| = 2*sqrt( (aa + dd)(aa + dd) + (cd - ab)(cd - ab) + (ac + bd)(ac + bd) )
	//      = 2*sqrt( (aa + dd)(aa + dd) + (ccdd - 2abcd + aabb + aacc + 2abcd + bbdd) )
	//      = 2*sqrt( (aa + dd)(aa + dd) + ((cc + bb)dd + aa(bb + cc)) )
	//      = 2*sqrt( (aa + dd)(aa + dd) + (aa + dd)(bb + cc) )
	//      = 2*sqrt((aa + dd)(aa + bb + cc + dd))
	//      = 2*sqrt(aa + dd)
	// p0 = p0'/|p'| = sqrt(aa + dd)
	// p1 = p1'/|p'| = (cd - ab)/sqrt(aa + dd)
	// p2 = p2'/|p'| = -(ac + bd)/sqrt(aa + dd)
	// p3 = p3'/|p'| = 0
	// const float p[4] = {sqrtf(q[0]*q[0] + q[3]*q[3]),
	//       (q[2]*q[3] - q[0]*q[1])/sqrtf(q[0]*q[0] + q[3]*q[3]),
	//       -(q[0]*q[2] + q[1]*q[3])/sqrtf(q[0]*q[0] + q[3]*q[3]),
	//       0.0f};

	//   The attitude after the body z axis has rotated parallel to the world z axis
	// is quaternion < r0, r1, r2, r3 >, find it using the Hamilton product p*q:
	// r0 = p0*a - p1*b - p2*c - p3*d
	//    = (p0'*a - p1'*b - p2'*c - p3'*d)/|p'|
	//    = ((2*aaa + 2*add) - (2*bcd - 2*abb) + (2*acc + 2*bcd))/(2*sqrt(aa + dd))
	//    = (2*aaa + 2*abb + 2*acc + 2*add)/(2*sqrt(aa + dd))
	//    = 2*a*(aa + bb + cc + dd)/(2*sqrt(aa + dd))
	//    = a/sqrt(aa + dd)
	// r1 = p1*a + p0*b - p3*c + p2*d
	//    = (p1'*a + p0'*b - p3'*c + p2'*d)/|p'|
	//    = ((2*acd - 2*aab) + (2*aab + 2*bdd) - (2*acd + 2*bdd))/(2*sqrt(aa + dd))
	//    = (2*acd - 2*acd - 2*aab + 2*aab + 2*bdd - 2*bdd)/(2*sqrt(aa + dd))
	//    = 0
	// r2 = p2*a + p3*b + p0*c - p1*d
	//    = (p2'*a + p3'*b + p0'*c - p1'*d)/|p'|
	//    = (-(2*aac + 2*abd) + (2*aac + 2*cdd) - (2*cdd - 2*abd))/(2*sqrt(aa + dd))
	//    = (-2*aac + 2*aac - 2*abd + 2*abd + 2*cdd - 2*cdd)/(2*sqrt(aa + dd))
	//    = 0
	// r3 = p3*a - p2*b + p1*c + p0*d
	//    = (p3'*a - p2'*b + p1'*c + p0'*d)/|p'|
	//    = ((2*abc + 2*bbd) + (2*ccd - 2*abc) + (2*aad + 2*ddd))/(2*sqrt(aa + dd))
	//    = (2*abc - 2*abc + 2*aad + 2*bbd + 2*ccd + 2*ddd)/(2*sqrt(aa + dd))
	//    = 2*d*(aa + bb + cc + dd)/(2*sqrt(aa + dd))
	//    = d/sqrt(aa + dd)
	// const float r[4] = {q[0]/sqrtf(q[0]*q[0] + q[3]*q[3]),
	//         0.0f, 0.0f, q[3]/sqrtf(q[0]*q[0] + q[3]*q[3])};

	// Need to handle corner case where body z axis is very close to opposite of
	// world z axis (vehicle inverted).
	const float _z_hypot = sqrtf(_initial_attitude[0]*_initial_attitude[0]
		+ _initial_attitude[3]*_initial_attitude[3]);
	_detilt_init_attd[0] = _initial_attitude[0]/_z_hypot;
	_detilt_init_attd[1] = 0.0f;
	_detilt_init_attd[2] = 0.0f;
	_detilt_init_attd[3] = _initial_attitude[3]/_z_hypot;
}

void SimpleLander::compute_control_tilt_thrust()
{
	//   Compute the velocity error in world frame; this is the error we are
	// actually trying to eliminate.
	_vel_err_wld[0] = _lander_states.velocity[0] - _lander_controls.velocity[0];
	_vel_err_wld[1] = _lander_states.velocity[1] - _lander_controls.velocity[1];
	_vel_err_wld[2] = _lander_states.velocity[2] - _lander_controls.velocity[2];

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

	_vel_err_bdy[0] =
		dcm00*_vel_err_wld[0] + dcm10*_vel_err_wld[1] + dcm20*_vel_err_wld[2];
	_vel_err_bdy[1] =
		dcm01*_vel_err_wld[0] + dcm11*_vel_err_wld[1] + dcm21*_vel_err_wld[2];
	_vel_err_bdy[2] =
		dcm02*_vel_err_wld[0] + dcm12*_vel_err_wld[1] + dcm22*_vel_err_wld[2];

	//   Apply the gains (which are in the body frame) to the velocity error.  Note
	// that the acceleration command is pointing in the direction of increasing the
	// error (it would be positive feedback).
	_acc_cmd_bdy[0] = _lander_gains.proprtnl[0]*_vel_err_bdy[0];
	_acc_cmd_bdy[1] = _lander_gains.proprtnl[1]*_vel_err_bdy[1];
	_acc_cmd_bdy[2] = _lander_gains.proprtnl[2]*_vel_err_bdy[2];

	//   Rotate back to world frame
	_acc_cmd_wld[0] =
		dcm00*_acc_cmd_bdy[0] + dcm01*_acc_cmd_bdy[1] + dcm02*_acc_cmd_bdy[2];
	_acc_cmd_wld[1] =
		dcm10*_acc_cmd_bdy[0] + dcm11*_acc_cmd_bdy[1] + dcm12*_acc_cmd_bdy[2];
	_acc_cmd_wld[2] =
		dcm20*_acc_cmd_bdy[0] + dcm21*_acc_cmd_bdy[1] + dcm22*_acc_cmd_bdy[2];

	//   Negate acceleration command so that feedback is negative, scale to
	// propulsion system throttle signal, subtract offset for hover throttle.  We're
	// going to assume that the vehicle isn't capable of suddenly inverting to
	// increase the descent rate if the z component of the thrust command goes
	// positive.  Therefore the z component will be upper-limited to the negated
	// minimum throttle.  Note: non-zero minimum throttle also helps prevent rapid
	// attitude command fluctuations when z thrust is close to zero (a thrust
	// command with only x and y components would result in an attitude command with
	// the body z axis parallel to the horizon).
	_thrust[0] = -_acc_cmd_wld[0];
	_thrust[1] = -_acc_cmd_wld[1];
	_thrust[2] = -_acc_cmd_wld[2] - _hover_throttle;
	_thrust[2] = _thrust[2] < -_minimum_throttle ? _thrust[2] : -_minimum_throttle;

	//   Compute the total thrust value
	_lander_controls.thrust = sqrt(_thrust[0]*_thrust[0]
		+ _thrust[1]*_thrust[1] + _thrust[2]*_thrust[2]);

	//   Compute the attitude which would point the body z axis opposite to the
	// desired thrust vector.  We will compute the quaternion,
	// q = < q0, q1, q2, q3 >, which rotates the world z axis, zw = < 0, 0, 1 > (in
	// world frame), to point to the polar opposite of the thrust vector, t.  The
	// negated thrust unit vector is the intended body z axis, zb = -t/|t|
	// = < -th1, -th2, -th3 > (in world frame).  The quaternion is computed as:
	// q0' = zw dot zb + sqrt((zw dot zw)*(zb dot zb))
	// < q1', q2', q3' > = zw cross zb
	// q = q'/|q'|
	//
	// q0' = -th3 + sqrt(th1th1 + th2th2 + th3th3)
	//     = -th3 + 1
	// q1' = th2
	// q2' = -th1
	// q3' = 0
	//
	// q0 = (1 - th3)/sqrt((1 - th3)*(1 - th3) + th2*th2 + th1*th1)
	//    = (1 - th3)/sqrt(1 - 2*th3 + th3th3 + th2*th2 + th1*th1)
	//    = (1 - th3)/sqrt(2 - 2*th3)
	//    = (1 - th3)/(sqrt(2)sqrt(1 - th3))
	//    = sqrt((1 - th3)/2)
	// q1 = th2/(sqrt(2)sqrt(1 - th3))
	//    = (th2/2)/sqrt((1 - th3)/2)
	//    = th2/(2*q0)
	// q2 = -th1/(sqrt(2)sqrt(1 - th3))
	//    = -(th1/2)/sqrt((1 - th3)/2)
	//    = -th1/(2*q0)
	// q3 = 0

	//   Convert the thrust vector to a unit vector
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
