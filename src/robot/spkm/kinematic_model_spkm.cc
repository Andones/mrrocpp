/*!
 * @file
 * @brief File containing definition of kinematic_model_spkm class methods.
 *
 * @author tkornuta
 * @date Jan 05, 2010
 *
 * @ingroup KINEMATICS SIF_KINEMATICS spkm
 */

#include <cmath>

#include "base/lib/com_buf.h"
#include "robot/spkm/exceptions.h"
#include "robot/spkm/kinematic_model_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

kinematic_model_spkm::kinematic_model_spkm(void)
{
	// Set model name.
	set_kinematic_model_label("SPKM kinematic model by D.Zlatanow and M.Zoppi");
}

void kinematic_model_spkm::check_motor_position(const lib::MotorArray & motor_position) const
{
	// Check upper limit for every motor.
	for (int i = 0; i < 3; ++i) {
		if (motor_position[i] > params.upper_motor_pos_limits[i])
			BOOST_THROW_EXCEPTION(spkm_motor_limit_error() << spkm_desired_value(motor_position[i]) << spkm_motor_number(i) << spkm_limit_type(UPPER_LIMIT));
		else if (motor_position[i] < params.lower_motor_pos_limits[i])
			BOOST_THROW_EXCEPTION(spkm_motor_limit_error() << spkm_desired_value(motor_position[i]) << spkm_motor_number(i) << spkm_limit_type(LOWER_LIMIT));
	}
}

void kinematic_model_spkm::check_joints(const lib::JointArray & q) const
{
	// Check joint limit for every axis.
	for (int i = 0; i < 3; ++i) {
		if (q[i] > params.upper_joints_limits[i])
			BOOST_THROW_EXCEPTION(spkm_joint_limit_error() << spkm_desired_value(q[i]) << spkm_joint_number(i) << spkm_limit_type(UPPER_LIMIT));
		else if (q[i] < params.lower_joints_limits[i])
			BOOST_THROW_EXCEPTION(spkm_joint_limit_error() << spkm_desired_value(q[i]) << spkm_joint_number(i) << spkm_limit_type(LOWER_LIMIT));
	}
}

void kinematic_model_spkm::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{
	// Precondition - check whether the desired position is valid.
	check_joints(local_desired_joints);

	// Compute desired motor positions for linear axes.
	for (int i = 0; i < 3; ++i) {
		local_desired_motor_pos_new[i] = (params.synchro_positions[i] - local_desired_joints[i])
				/ params.mp2i_ratios[i];
	}

	// Compute desired motor positions for rotary axes.
	for (int i = 3; i < 6; ++i) {
		local_desired_motor_pos_new[i] = local_desired_joints[i] * 4 * 2000 * 100 / (2 * M_PI);
	}

	// Postcondition
	check_motor_position(local_desired_motor_pos_new);
}

void kinematic_model_spkm::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{
	// Precondition
	check_motor_position(local_current_motor_pos);

	// Linear axes
	for (int i = 0; i < 3; ++i) {
		// Add different translation (in mm) depending on axis number (A=0, B=1, C=2).
		local_current_joints[i] = params.synchro_positions[i] - local_current_motor_pos[i] * params.mp2i_ratios[i];
	}

	// Rotary axes
	for (int i = 3; i < 6; ++i) {
		local_current_joints[i] = ((local_current_motor_pos[i] / (4 * 2000)) / 100) * 2 * M_PI;
	}

	// Postcondition
	check_joints(local_current_joints);
}


void kinematic_model_spkm::inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{
	// Transform Homog_matrix to Matrix4d.
	Homog4d O_W_T_desired;
	O_W_T_desired << local_desired_end_effector_frame(0, 0), local_desired_end_effector_frame(0, 1), local_desired_end_effector_frame(0, 2), local_desired_end_effector_frame(0, 3), local_desired_end_effector_frame(1, 0), local_desired_end_effector_frame(1, 1), local_desired_end_effector_frame(1, 2), local_desired_end_effector_frame(1, 3), local_desired_end_effector_frame(2, 0), local_desired_end_effector_frame(2, 1), local_desired_end_effector_frame(2, 2), local_desired_end_effector_frame(2, 3), 0, 0, 0, 1;
    std::cout <<"Desired pose of the end-effector:\n" << O_W_T_desired<<std::endl;

    // Compute the required O_S_T - pose of the spherical wrist middle (S) in global reference frame (O).
	Homog4d  O_S_T_desired = O_W_T_desired * params.W_S_T;
	std::cout <<"Desired pose of the wrist center:\n" << O_S_T_desired<<std::endl;

    // Compute e basing only on translation O_S_P from O_S_T.
	Vector5d e = PM_S_to_e(O_S_T_desired);

	// Compute inverse PM kinematics basing on e.
	Vector3d PKM_joints = PM_inverse_from_e(e);

	// Compute upper platform pose.
	Homog4d O_P_T = PM_O_P_T_from_e(e);
	std::cout <<"Computed upper platform pose:\n" << O_P_T << std::endl;

	// Compute the pose of wrist (S) on the base of upper platform pose.
	Homog4d O_S_T_computed;
	O_S_T_computed = O_P_T * params.P_S_T;
	std::cout <<"Computed pose of wrist center:\n" << O_S_T_computed << std::endl;

	// Compute the desired "twist of the wrist".
	// Transformation from computed OST to desired OST.
	Homog4d wrist_twist = O_S_T_desired.inverse()*O_S_T_computed;
	std::cout <<"Twist:\n" << O_S_T_computed << std::endl;

	// Compute the inverse transform of the spherical wrist basing on its "twist".
	//thetas = SW_inverse(obj, S_S_prim_R);

	// Fill joints array.
	/*	local_desired_joints[0] = PKM_joints[0];
	 local_desired_joints[1] = PKM_joints[1];
	 local_desired_joints[2] = PKM_joints[2];
	 local_desired_joints[3] = SW_joints[0];
	 local_desired_joints[4] = SW_joints[1];
	 local_desired_joints[5] = SW_joints[2];*/
}

Vector5d kinematic_model_spkm::PM_S_to_e(const Homog4d & O_S_T_)
{
	// Extract variables describing position of the middle of wrist
	double x = O_S_T_(0 ,3);
	double y = O_S_T_(1,3);
	double z = O_S_T_(2,3);

	std::cout<<"PM_S_to_e: ["<<x<<", "<<y<<", "<<z<<"]\n";

/*
            % Temporary variables used for computations of alpha.
            t0_sq = O_S_P(1) * O_S_P(1) + O_S_P(3) * O_S_P(3);
            hx_sq = obj.params.P_S_P(1) * obj.params.P_S_P(1);

            % Compute sine and cosine of the alpha angle.
            s_alpha = (delta_B1 * O_S_P(3) * sqrt(t0_sq - hx_sq) + obj.params.P_S_P(1) * O_S_P(1)) / (t0_sq);
            c_alpha = (-delta_B1 * O_S_P(1) * sqrt(t0_sq - hx_sq) + obj.params.P_S_P(1) * O_S_P(3)) / (t0_sq);

            % Compute sine and cosine of the beta angle.
            t6 = (delta_B1 * (t0_sq - obj.params.dB * O_S_P(1)) * sqrt(t0_sq - hx_sq) + ...
                + obj.params.dB * obj.params.P_S_P(1) * O_S_P(3)) / (t0_sq);

            s_beta = -delta_B2 * O_S_P(2) / sqrt(t6 * t6 + O_S_P(2) * O_S_P(2));
            c_beta = delta_B2 * t6 / sqrt(t6 * t6 + O_S_P(2) * O_S_P(2));

            % Compute h.
            h = delta_B2 * (O_S_P(2) * O_S_P(2) + delta_B1 * t6 * sqrt(t0_sq - hx_sq)) / sqrt(t6 * t6 + O_S_P(2) * O_S_P(2)) - obj.params.P_S_P(3);

 */


	// Temporary variables used for computations of alpha.
	double t0_sq = x * x + z * z;
	double hx_sq = params.P_S_P.x() * params.P_S_P.x();

	// Compute sine and cosine of the alpha angle.
	double s_alpha = (z * sqrt(t0_sq - hx_sq) + x * params.P_S_P.x()) / (t0_sq);
	double c_alpha = (-x * sqrt(t0_sq - hx_sq) + z * params.P_S_P.x()) / (t0_sq);

	// Compute sine and cosine of the beta angle.
	double t6 = ((z * z + x * x - params.dB * x) * sqrt(t0_sq - hx_sq) + params.dB * z * params.P_S_P.x()) / (t0_sq);
	double s_beta = -y / sqrt(t6 * t6 + y * y);
	double c_beta = t6 / sqrt(t6 * t6 + y * y);

	// Compute h.
	double h = (y * y + t6 * sqrt(t0_sq - hx_sq)) / sqrt(t6 * t6 + y * y) - params.P_S_P.z();

	// Return computed e vector.*/
	Vector5d e;
	e << s_alpha, c_alpha, s_beta, c_beta, h;
	std::cout<<"e= ["<<e.inverse()<<"]\n";

	return e;
}

Vector3d kinematic_model_spkm::PM_inverse_from_e(const Vector5d & e_)
{
	// "Retrieve" values from e.
	double s_alpha = e_[0];
	double c_alpha = e_[1];
	double s_beta = e_[2];
	double c_beta = e_[3];
	double h = e_[4];

	// Compute temporary variables.
	double t1 = params.dB * s_alpha - params.pB;
	double t2 = params.dB * c_alpha * c_beta + h;
	double t3 = params.dB * c_alpha - t2 * c_beta;
	// Compute joints.
	double qB = sqrt(t1 * t1 + t2 * t2);
	double qA = sqrt(pow(t3 - params.pA * s_beta, 2.0) + pow(t2 * s_beta - params.pA * c_beta + params.dA, 2.0));
	double qC = sqrt(pow(t3 - params.pC * s_beta, 2.0) + pow(t2 * s_beta - params.pC * c_beta + params.dC, 2.0));

	// Return vector with joints.*/
	Vector3d joints;
	joints << qA, qB, qC;

	std::cout<<"joints: "<<joints<<std::endl;

	return joints;
}

Homog4d kinematic_model_spkm::PM_O_P_T_from_e(const Vector5d & e_)
{
	Matrix4d O_P_T;

	// "Retrieve" values from e.
	double s_alpha = e_[0];
	double c_alpha = e_[1];
	double s_beta = e_[2];
	double c_beta = e_[3];
	double h = e_[4];

	// Compute matrix representing the location and orientation of upper platform.
	O_P_T(0, 0) = s_alpha;
	O_P_T(0, 1) = 0.0;
	O_P_T(0, 2) = c_alpha;
	O_P_T(0, 3) = 0.0;
	O_P_T(1, 0) = -s_beta * c_alpha;
	O_P_T(1, 1) = c_beta;
	O_P_T(1, 2) = s_beta * s_alpha;
	O_P_T(1, 3) = c_alpha * params.dB * s_beta;
	O_P_T(2, 0) = -c_beta * c_alpha;
	O_P_T(2, 1) = -s_beta;
	O_P_T(2, 2) = c_beta * s_alpha;
	O_P_T(2, 3) = -h;
	O_P_T(3, 0) = 0.0;
	O_P_T(3, 1) = 0.0;
	O_P_T(3, 2) = 0.0;
	O_P_T(3, 3) = 1.0;

	// Return matrix.
	return Homog4d(O_P_T);
}

Vector3d kinematic_model_spkm::SW_inverse(const Homog4d & P_W_T_)
{
	// TODO Inverse kinematics of the spherical wrist.

	// Return vector with joints.
	Vector3d joints;
	joints << 0, 0, 0;
	return joints;
}

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp
