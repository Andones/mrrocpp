/*!
 * @file mp_t_smb_powered_from_bench_test.h
 * @brief Class for SMB tests.
 *
 * @date Jan 17, 2012
 * @author tkornuta
 */

#if !defined(__MP_T_SWARMITFIX_DEMO_BASE_H)
#define __MP_T_SWARMITFIX_DEMO_BASE_H


#include "base/mp/mp_task.h"

// MP robot classes.
#include "robot/spkm/mp_r_spkm1.h"
#include "robot/spkm/mp_r_spkm2.h"
#include "robot/smb/mp_r_smb1.h"
#include "robot/smb/mp_r_smb2.h"
#include "robot/shead/mp_r_shead1.h"
#include "robot/shead/mp_r_shead2.h"
#include "robot/sbench/mp_r_sbench.h"

#include "taks_utils.hpp"


namespace mrrocpp {
namespace mp {
namespace task {
namespace swarmitfix {


/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  @{
 */

/*!
 * @brief Base class for all test and demo SwarmItFIX tasks.
 * Contains methods facilitating the control of different type of motions for different agents (SMB, SPKM, SHEAD).
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 * @date Jan 17, 2012
 */
class demo_base : public mrrocpp::mp::task::task
{

protected:
	/*!
	 * Sends motor rotation command to SMB in the joint space.
	 * @param [in] legs_rotation_ Desired absolute rotation around leg (in external values -6, -5, ..., 5, 6).
	 * @param [in] pkm_rotation_ Desired absolute rotation of the upper SMP by given angle [radians].
	 */
	void smb_rotate_external(const lib::robot_name_t & robot_name, int legs_rotation_, double pkm_rotation_);

	/*!
	 * Moves SMB legs in and out.
	 * @param [in] l1_ Desired position of the leg one (in, out).
	 * @param [in] l2_ Desired position of the leg two (in, out).
	 * @param [in] l3_ Desired position of the leg three (in, out).
	 */
	void smb_pull_legs(const lib::robot_name_t & robot_name, lib::smb::FESTO_LEG l1_, lib::smb::FESTO_LEG l2_, lib::smb::FESTO_LEG l3_);

	/*!
	 * @brief Stands on given leg (this one stays out, rest goes in).
	 *
	 * @param [in] leg_number_ Leg around which the rotation will be performed.
	 */
	void smb_stan_on_one_leg(const lib::robot_name_t & robot_name, int leg_number_);

	/*!
	 * @brief Rotates agent around given leg, thus realizes the sequence: pull two legs in, rotate around the third one and pull all legs out.
	 *
	 * @note Desired start as well as final states are 'all legs out'.
	 * @note Utilizes the move_smb_legs() and move_smb_external() methods.
	 * @note PKM rotation is set to zero.
	 *
	 * @param [in] leg_number_ Leg around which the rotation will be performed.
	 * @param [in] rotation_ Rotation of the legs (in external values -6, -5, ..., 5, 6).
	 */
	void rotate_smb(const lib::robot_name_t & robot_name, int leg_number_, int rotation_);

	/*!
	 * Controls the head rotation.
	 *
	 * @param [in] joint_ Desired absolute position in the joint space.
	 */
	void move_shead_joints(const lib::robot_name_t & robot_name, double joint_);

	/*!
	 * Controls the head vacuum.
	 *
	 * @param [in] enabled state of the vacuum.
	 */
	void shead_vacuum(const lib::robot_name_t & robot_name, bool enabled);

	/*!
	 * Controls the head solidification.
	 *
	 * @param [in] enabled state of the solidification.
	 */
	void shead_solidify(const lib::robot_name_t & robot_name, bool enabled);

	/*!
	 * Moves the PKM to the desired position in the joint space
	 *
	 * @param [in] motion_variant_ Variant of the motion to be executed (here only NON_SYNC_TRAPEZOIDAL, SYNC_TRAPEZOIDAL are available).
	 */
	void move_spkm_joints(const lib::robot_name_t & robot_name, mrrocpp::lib::epos::EPOS_MOTION_VARIANT motion_variant_, double legA_, double legB_, double legC_, double wrist1_, double wrist2_, double wrist3_);

	/*!
	 * Moves the PKM to the desired pose in the cartesian space.
	 *
	 * @param [in] motion_variant_ Variant of the motion to be executed (here NON_SYNC_TRAPEZOIDAL, SYNC_TRAPEZOIDAL, OPERATIONAL are available).
	 */
	void move_spkm_external(const lib::robot_name_t & robot_name, mrrocpp::lib::epos::EPOS_MOTION_VARIANT motion_variant_, const lib::Xyz_Euler_Zyz_vector & pose_);

	/*!
	 * Brake the moog motor.
	 */
	void spkm_brake(const lib::robot_name_t & robot_name);

	/*!
	 * @brief Method responsible for supporting the plate in give point and return.
	 *  The trajectory is acquired though an intermediate pose (the same intermediate pose is considered in both directions).
	 *
	 * @author tkornuta
	 * @param support_pose_ - xyz_zyz of support pose.
	 * @param inter_pose_ - xyz_zyz of intermediate pose.
	 * @param smb_joint_ - rotation of the SMB (the motor rotating the upper SMB plate).
	 * @param shead_joint - rotation of the SHEAD.
	 */
	void move_to_pose_and_return(const lib::robot_name_t & robot_name,
			const lib::Xyz_Euler_Zyz_vector & support_pose_,
			const lib::Xyz_Euler_Zyz_vector & inter_pose_,
			double smb_joint_, double shead_joint_);

	/*!
	 * Controls the bench power supply.
	 */
	void control_bench_power_supply(const mrrocpp::lib::sbench::power_supply_state & ps_, int delay_);

	/*!
	 * Controls the bench cleaning.
	 */
	void control_bench_cleaning(const mrrocpp::lib::sbench::cleaning_state & cs_, int delay_);

	/*!
	 * Controls the bench power (the leg smb is not controlled, thus rotation is simulated).
	 */
	void bench_execute_power_move(const power_smb_move & move_, unsigned int delay_);

	/*!
	 * Controls the bench power and cleaning (the leg smb is not controlled, thus rotation is simulated).
	 */
	void bench_execute_power_move_with_cleaning(const power_smb_move & move_, unsigned int delay_, unsigned int cleaning_time_);

	/*!
	 * Controls the bench power with rotation of the smb leg.
	 */
	void smb_execute_power_move(const lib::robot_name_t & robot_name, const power_smb_move & move_, unsigned int delay_);

	/*!
	 * Controls the bench power and cleaning with rotation of the smb leg.
	 */
	void smb_execute_power_move_with_cleaning(const lib::robot_name_t & robot_name, const power_smb_move & move_, unsigned int delay_, unsigned int cleaning_time_);


public:
	//! Calls the base class constructor.
	demo_base(lib::configurator &config_);

	//! Empty.
	virtual ~demo_base() { }

};

/** @} */ // end of swarmitfix
} /* namespace swarmitfix */
}// namespace task
} // namespace mp
} // namespace mrrocpp

#endif
