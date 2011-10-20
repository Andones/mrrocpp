#include <cstdio>
#include <iostream>
#include <bitset>
#include <boost/foreach.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp_e_smb.h"
#include "festo_and_inputs.h"

#include "base/edp/reader.h"
// Kinematyki.
#include "robot/smb/kinematic_model_smb.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "exceptions.h"

using namespace std;

namespace mrrocpp {
namespace edp {
namespace smb {

/*
 TODO: Set propper values of A and V.
 const uint32_t effector::Vdefault[lib::smb::NUM_OF_SERVOS] = { 100UL, 100UL };
 const uint32_t effector::Adefault[lib::smb::NUM_OF_SERVOS] = { 1000UL, 1000UL };
 const uint32_t effector::Ddefault[lib::smb::NUM_OF_SERVOS] = { 1000UL, 1000UL };
 */

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::single_thread_master_order(nm_task, nm_tryb);
}

void effector::check_controller_state()
{
	if (robot_test_mode)
		return;

	// Try to get state of each axis
	unsigned int powerOn = 0;
	unsigned int notInFaultState = 0;
	// Check axes.
	for (size_t i = 0; i < axes.size(); ++i) {
		try {
			// Print state.
			axes[i]->printState();
			// Get current epos state.
			maxon::epos::actual_state_t state = axes[i]->getState();
			// Check if in the FAULT state
			if (state == maxon::epos::FAULT) {
				// Read number of errors
				int errNum = axes[i]->getNumberOfErrors();
				for (int j = 1; j <= errNum; ++j) {
					// Get the detailed error
					uint32_t errCode = axes[i]->getErrorHistory(j);
					// Send message to SR.
					msg->message(mrrocpp::lib::FATAL_ERROR, string("Axis ") + axesNames[i] + ": "
							+ axes[i]->ErrorCodeMessage(errCode));
				}
			} else if (state == maxon::epos::SWITCH_ON_DISABLED) {
				// Send message to SR.
				msg->message(mrrocpp::lib::FATAL_ERROR, string("Axis ") + axesNames[i] + " is in the Disabled state");
			} else {
				notInFaultState++;
			}
			// In this loop step we are sure that axis is working, this powered.
			powerOn++;
		} catch (...) {
			// Probably the axis is not powered on, do nothing.
		}
	}
	// Robot is synchronised if only one axis - the one controlling the PKM rotation - is referenced.
	controller_state_edp_buf.is_synchronised = axes[1]->isReferenced();
	// Check whether robot is powered.
	controller_state_edp_buf.is_power_on = (powerOn == axes.size());
	// Check fault state.
	controller_state_edp_buf.robot_in_fault_state = (notInFaultState != axes.size());
}

void effector::get_controller_state(lib::c_buffer &instruction)
{
	// False is the initial value
	controller_state_edp_buf.is_synchronised = false;
	controller_state_edp_buf.is_power_on = false;
	controller_state_edp_buf.robot_in_fault_state = false;

	// Check controller state.
	check_controller_state();

	// Copy data to reply buffer
	reply.controller_state = controller_state_edp_buf;

	// Check if it is safe to calculate joint positions
	if (is_synchronised()) {
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
	}

	// Lock data structure during update
	{
		boost::mutex::scoped_lock lock(effector_mutex);

		// Initialize internal data
		for (int i = 0; i < number_of_servos; i++) {
			servo_current_motor_pos[i] = desired_motor_pos_new[i] = desired_motor_pos_old[i] = current_motor_pos[i];
			desired_joints[i] = current_joints[i];
		}
	}
}

// Konstruktor.
effector::effector(common::shell &_shell, lib::robot_name_t l_robot_name) :
		motor_driven_effector(_shell, l_robot_name)
{

	number_of_servos = lib::smb::NUM_OF_SERVOS;
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();

	if (!robot_test_mode) {
		// Create gateway object.
		if (this->config.exists("can_iface")) {
			gateway =
					(boost::shared_ptr <canopen::gateway>) new canopen::gateway_socketcan(config.value <std::string>("can_iface"));
		} else {
			gateway = (boost::shared_ptr <canopen::gateway>) new canopen::gateway_epos_usb();
		}

		// Connect to the gateway.
		gateway->open();

		// Create epos objects according to CAN ID-mapping.
		epos_di_node = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 8);
		pkm_rotation_node = (boost::shared_ptr <maxon::epos>) new maxon::epos(*gateway, 9);

		// Collect axes into common array container.
		axes[0] = &(*epos_di_node);
		axesNames[0] = "legs";
		axes[1] = &(*pkm_rotation_node);
		axesNames[1] = "pkm";

		// Create festo node.
		cpv10 = (boost::shared_ptr <festo::cpv>) new festo::cpv(*gateway, 10);
	}

}

void effector::reset_variables()
{
	// Zero all variables related to motor positions.
	for (int i = 0; i < number_of_servos; ++i) {
		current_motor_pos[i] = 0;
	}
	desired_motor_pos_old = current_motor_pos;
	desired_motor_pos_new = current_motor_pos;

	// Compute current motor positions on the base of zeroed motors.
	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);
	desired_joints = current_joints;
}

void effector::synchronise(void)
{
	if (robot_test_mode) {
		controller_state_edp_buf.is_synchronised = true;
		return;
	}

	try {
		// Two-step synchronization of the motor rotating the whole PKM.
		// Step1: Potentiometer.
		// Get current potentiometer readings.
		int pot = pkm_rotation_node->getAnalogInput1();

		// Set coefficients.
		const double p1 = -0.0078258336;
		const double p2 = 174.7796278191;
		const double p3 = -507883.404901415;

		// Compute desired position.
		int position = -(pot * pot * p1 + pot * p2 + p3) - 120000;
		cout << "Retrieved potentiometer reading: " << pot << "\nComputed pose: " << position << endl;

		// Move to the relative position.
		pkm_rotation_node->moveRelative(position);
		while (!pkm_rotation_node->isTargetReached())
			usleep(200000);

		// Step2: Homing.
		// Activate homing mode.
		pkm_rotation_node->doHoming(maxon::epos::HM_INDEX_NEGATIVE_SPEED, 0);

		// Compute joints positions in the home position
		get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

		// Now the robot is synchronised
		//	controller_state_edp_buf.is_synchronised = true;
		// Check whether the synchronization was successful.
		check_controller_state();

	} catch (mrrocpp::lib::exception::mrrocpp_non_fatal_error & e_) {
		// Standard error handling.
		HANDLE_MRROCPP_ERROR(e_)
	} catch (mrrocpp::lib::exception::mrrocpp_fatal_error & e_) {
		// Standard error handling.
		HANDLE_MRROCPP_ERROR(e_)
	} catch (mrrocpp::lib::exception::mrrocpp_system_error & e_) {
		// Standard error handling.
		HANDLE_MRROCPP_ERROR(e_)
	} catch (...) {
		msg->message(mrrocpp::lib::FATAL_ERROR, "Unknown error");
	}

}

lib::smb::ALL_LEGS_VARIANT effector::current_legs_state(void)
{
	return fai->current_legs_state;
}

lib::smb::ALL_LEGS_VARIANT effector::next_legs_state(void)
{
	return fai->next_legs_state;
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction)
{
	try {
		msg->message("move_arm");

		switch (ecp_edp_cbuffer.variant)
		{
			case lib::smb::POSE: {
				msg->message("POSE");

				switch (ecp_edp_cbuffer.set_pose_specification)
				{
					case lib::smb::MOTOR: {
						msg->message("MOTOR");
						// Copy data directly from buffer
						rotational_motors_command();
					}
						break;
					case lib::smb::JOINT: {
						msg->message("JOINT");
					}
						break;
					case lib::smb::FRAME: {
						msg->message("FRAME");
					}
						break;
				}
				// Control the two SMB rotational motors.

				break;
			}
			case lib::smb::QUICKSTOP: {
				msg->message("QUICKSTOP");
				if (!robot_test_mode) {
					// Execute command
					BOOST_FOREACH(maxon::epos * node, axes)
							{
								// Brake with Quickstop command
								node->setState(maxon::epos::QUICKSTOP);
							}
				}
				break;
			}
			case lib::smb::CLEAR_FAULT: {
				msg->message("CLEAR_FAULT");
				BOOST_FOREACH(maxon::epos * node, axes)
						{
							// Print state.
							node->printState();
							// Check if node is in a FAULT state.
							if (node->getState() == maxon::epos::FAULT) {
								maxon::UNSIGNED8 errNum = node->getNumberOfErrors();
								cerr << "readNumberOfErrors() = " << (int) errNum << endl;
								// Print list of errors.
								for (maxon::UNSIGNED8 i = 1; i <= errNum; ++i) {
									maxon::UNSIGNED32 errCode = node->getErrorHistory(i);
									cerr << node->ErrorCodeMessage(errCode) << endl;
								}
								// Clear errors.
								if (errNum > 0) {
									node->clearNumberOfErrors();
								}
								// Reset errors.
								node->setState(maxon::epos::FAULT_RESET);
							}
							// Reset node.
							node->reset();
						}
				break;
			}
			case lib::smb::FESTO: {
				if (is_base_positioned_to_move_legs) {
					fai->command();
				}
				break;
			}
			default:
				break;

		}
	} catch (mrrocpp::lib::exception::mrrocpp_non_fatal_error & e_) {
		// Standard error handling.
		HANDLE_MRROCPP_ERROR(e_)
	} catch (mrrocpp::lib::exception::mrrocpp_fatal_error & e_) {
		// Standard error handling.
		HANDLE_MRROCPP_ERROR(e_)
	} catch (mrrocpp::lib::exception::mrrocpp_system_error & e_) {
		// Standard error handling.
		HANDLE_MRROCPP_ERROR(e_)
	} catch (...) {
		msg->message(mrrocpp::lib::FATAL_ERROR, "Unknown error");
	}
}

void effector::rotational_motors_command()
{
	/*	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	 ss << ecp_edp_cbuffer.motor_pos[1];
	 msg->message(ss.str().c_str());
	 ss << ecp_edp_cbuffer.set_pose_specification;
	 msg->message(ss.str().c_str());*/

	if (current_legs_state() != lib::smb::TWO_UP_ONE_DOWN) {
		// The TWO_UP_ONE_DOWN is the only state in which control of both motors (legs and SPKM rotations) is possible.
		// In other states control of the motor rotating the legs (lower SMB motor) is prohibited!

		// Check the difference between current and desired values.
		// Check motors.
		if ((ecp_edp_cbuffer.set_pose_specification == lib::smb::MOTOR)
				&& (current_motor_pos[0] != ecp_edp_cbuffer.motor_pos[0]))
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_clamps_rotation_prohibited_in_given_state()<<current_state(current_legs_state()));
		// Check joints.
		else if ((ecp_edp_cbuffer.set_pose_specification == lib::smb::JOINT)
				&& (current_joints[0] != ecp_edp_cbuffer.motor_pos[0]))
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_clamps_rotation_prohibited_in_given_state()<<current_state(current_legs_state()));
	}

	// TODO: remove this line!
	ecp_edp_cbuffer.set_pose_specification = lib::smb::MOTOR;

	// Interpret command according to the pose specification.
	switch (ecp_edp_cbuffer.set_pose_specification)
	{
		case lib::smb::MOTOR:
			// Copy data directly from buffer
			for (int i = 0; i < number_of_servos; ++i) {
				desired_motor_pos_new[i] = ecp_edp_cbuffer.motor_pos[i];
				cout << "MOTOR[ " << i << "]: " << desired_motor_pos_new[i] << endl;
			}
			// Check the desired motor (only motors!) values if they are absolute.
			get_current_kinematic_model()->check_motor_position(desired_motor_pos_new);
			// Transform desired motors to joints.
			get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints);
			break;
		default:
			// Throw non-fatal error - invalid pose specification.
			BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_pose_specification());
			break;
	} //: switch (ecp_edp_cbuffer.set_pose_specification)

	// TODO: remove this line!
	ecp_edp_cbuffer.motion_variant = lib::epos::NON_SYNC_TRAPEZOIDAL;

	// Perform motion depending on its type.
	// Note: at this point we assume, that desired_motor_pos_new holds a validated data.
	switch (ecp_edp_cbuffer.motion_variant)
	{
		case lib::epos::NON_SYNC_TRAPEZOIDAL:
			// Execute command.
			for (size_t i = 0; i < axes.size(); ++i) {
				if (is_synchronised()) {
					cout << "MOTOR: moveAbsolute[" << i << "] ( " << desired_motor_pos_new[i] << ")" << endl;
					if (!robot_test_mode) {
						/*						axes[i]->writeProfileVelocity(Vdefault[i]);
						 axes[i]->writeProfileAcceleration(Adefault[i]);
						 axes[i]->writeProfileDeceleration(Ddefault[i]);*/
						axes[i]->moveAbsolute(desired_motor_pos_new[i]);
					} else {
						current_joints[i] = desired_joints[i];
						current_motor_pos[i] = desired_motor_pos_new[i];
					}
				} else {
					cout << "MOTOR: moveRelative[" << i << "] ( " << desired_motor_pos_new[i] << ")" << endl;
					if (!robot_test_mode) {
						/*						axes[i]->writeProfileVelocity(Vdefault[i]);
						 axes[i]->writeProfileAcceleration(Adefault[i]);
						 axes[i]->writeProfileDeceleration(Ddefault[i]);*/
						axes[i]->moveRelative(desired_motor_pos_new[i]);
					} else {
						current_joints[i] += desired_joints[i];
						current_motor_pos[i] += desired_motor_pos_new[i];
					}
				}
			}
			break;
		default:
			// Throw non-fatal error - motion type not supported.
			BOOST_THROW_EXCEPTION(mrrocpp::edp::exception::nfe_invalid_motion_type());
			break;
	} //: switch (ecp_edp_cbuffer.motion_variant)

}

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	try {

		// Check controller state.
		check_controller_state();

		/*	std::stringstream ss(std::stringstream::in | std::stringstream::out);
		 ss << instruction.get_arm_type;
		 msg->message(ss.str().c_str());*/

		if (instruction.instruction_type != lib::SET) {
			switch (ecp_edp_cbuffer.get_pose_specification)
			{

				case lib::smb::MOTOR:
					msg->message("EDP get_arm_position MOTOR");
					for (size_t i = 0; i < number_of_servos; ++i) {
						if (robot_test_mode) {
							edp_ecp_rbuffer.epos_controller[i].position = current_motor_pos[i];
							edp_ecp_rbuffer.epos_controller[i].current = 0;
							edp_ecp_rbuffer.epos_controller[i].motion_in_progress = false;
						} else {
							current_motor_pos[i] = axes[i]->getActualPosition();
							edp_ecp_rbuffer.epos_controller[i].position = current_motor_pos[i];
							edp_ecp_rbuffer.epos_controller[i].current = axes[i]->getActualCurrent();
							edp_ecp_rbuffer.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
						}
					}
					break;
				case lib::smb::JOINT:
					msg->message("EDP get_arm_position JOINT");

					// Read actual values from hardware
					/*			if (!robot_test_mode) {
					 for (size_t i = 0; i < axes.size(); ++i) {
					 current_motor_pos[i] = axes[i]->readActualPosition();
					 }
					 }*/

					// Calculate current joint values.
					get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

					// Copy values to buffer.
					for (int i = 0; i < number_of_servos; ++i) {
						edp_ecp_rbuffer.epos_controller[i].position = current_joints[i];
					}
					break;
				case lib::smb::FRAME: {
					msg->message("EDP get_arm_position FRAME");

					//	edp_ecp_rbuffer.current_pose = lib::Homog_matrix();
				}
					break;
				default:
					break;

			}
		}

		// SMB clamps.
		fai->create_reply();
		reply.servo_step = step_counter;

	} catch (mrrocpp::lib::exception::mrrocpp_non_fatal_error & e_) {
		// Standard error handling.
		HANDLE_MRROCPP_ERROR(e_)
	} catch (mrrocpp::lib::exception::mrrocpp_fatal_error & e_) {
		// Standard error handling.
		HANDLE_MRROCPP_ERROR(e_)
	} catch (mrrocpp::lib::exception::mrrocpp_system_error & e_) {
		// Standard error handling.
		HANDLE_MRROCPP_ERROR(e_)
	} catch (...) {
		msg->message(mrrocpp::lib::FATAL_ERROR, "Unknown error");
	}
}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::smb::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

void effector::create_threads()
{
	fai = new festo_and_inputs(*this);
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);

	// do poprawy
	is_base_positioned_to_move_legs = true;

}

void effector::instruction_deserialization()
{
	memcpy(&ecp_edp_cbuffer, instruction.serialized_command, sizeof(ecp_edp_cbuffer));
}

void effector::reply_serialization(void)
{
	memcpy(reply.serialized_reply, &edp_ecp_rbuffer, sizeof(edp_ecp_rbuffer));
	assert(sizeof(reply.serialized_reply) >= sizeof(edp_ecp_rbuffer));
}

} // namespace smb
} // namespace edp
} // namespace mrrocpp

