#include <iostream>
#include <cstdio>

#include <boost/foreach.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "robot/spkm/edp_e_spkm.h"
#include "base/edp/reader.h"
//#include "base/edp/vis_server.h"

#include "robot/spkm/kinematic_model_spkm.h"
#include "robot/spkm/kinematic_parameters_spkm.h"
#include "base/edp/manip_trans_t.h"

#include "robot/epos/epos.h"
#include "robot/epos/epos_access_usb.h"
#include "base/lib/pvt.hpp"

#include "robot/spkm/exceptions.h"

namespace mrrocpp {
namespace edp {
namespace spkm {

const uint32_t effector::Vdefault[6] = { 5000UL, 5000UL, 5000UL, 5000UL, 5000UL, 5000UL };
const uint32_t effector::Adefault[6] = { 2000UL, 2000UL, 2000UL, 2000UL, 2000UL, 2000UL };
const uint32_t effector::Ddefault[6] = { 2000UL, 2000UL, 2000UL, 2000UL, 2000UL, 2000UL };

// Konstruktor.
effector::effector(lib::configurator &_config) :
	manip_effector(_config, lib::spkm::ROBOT_NAME)
{
	number_of_servos = lib::spkm::NUM_OF_SERVOS;

	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();

	if (!robot_test_mode) {
		// Create gateway object
		gateway = (boost::shared_ptr <epos::epos_access>) new epos::epos_access_usb();

		// Connect to the gateway
		gateway->open();

		// Create epos objects according to CAN ID-mapping
		axisA = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 5);
		axisB = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 4);
		axisC = (boost::shared_ptr <epos::epos>) new epos::epos(*gateway, 6);
		axis1 = (boost::shared_ptr<epos::epos>) new epos::epos(*gateway, 3);
		axis2 = (boost::shared_ptr<epos::epos>) new epos::epos(*gateway, 2);
		axis3 = (boost::shared_ptr<epos::epos>) new epos::epos(*gateway, 1);

		// Collect axes into common array container
		axes[0] = &(*axisA); axesNames[0] = "A";
		axes[1] = &(*axisB); axesNames[1] = "B";
		axes[2] = &(*axisC); axesNames[2] = "C";
		axes[3] = &(*axis1); axesNames[3] = "1";
		axes[4] = &(*axis2); axesNames[4] = "2";
		axes[5] = &(*axis3); axesNames[5] = "3";
	}
}

void effector::get_controller_state(lib::c_buffer &instruction)
{
	// False is the initial value
	controller_state_edp_buf.is_synchronised = false;
	controller_state_edp_buf.is_power_on = false;
	controller_state_edp_buf.is_robot_blocked = false;

	if (!robot_test_mode) {
		// Try to get state of each axis
		unsigned int referenced = 0;
		unsigned int powerOn = 0;
		unsigned int notInFaultState = 0;
		for (std::size_t i = 0; i < axes.size(); ++i)
		{
			try {
				// Check if in the FAULT state
				if (axes[i]->checkEPOSstate() == 11) {
					// Read number of errors
					int errNum = axes[i]->readNumberOfErrors();
					for (int j = 1; j <= errNum; ++j) {
						// Get the detailed error
						uint32_t errCode = axes[i]->readErrorHistory(j);

						msg->message(std::string("axis ") + axesNames[i] + ": " + epos::epos::ErrorCodeMessage(errCode));
					}
				} else {
					notInFaultState++;
				}
				if (axes[i]->isReferenced()) {
					// Do not break from this loop so this is a also a preliminary axis error check
					referenced++;
				}
				powerOn++;
			} catch (...) {
				// Probably the axis is not powered on, do nothing.
			}
		}
		// Robot is synchronised if all axes are referenced
		controller_state_edp_buf.is_synchronised = (referenced == axes.size());
		controller_state_edp_buf.is_power_on = (powerOn == axes.size());
		controller_state_edp_buf.is_robot_blocked = (notInFaultState == axes.size());
	}

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

void effector::synchronise(void)
{
	if (robot_test_mode) {
		controller_state_edp_buf.is_synchronised = true;
		return;
	}

	// reset controller
	BOOST_FOREACH(epos::epos * node, axes)
	{
		node->reset();
	}

	// switch to homing mode
	BOOST_FOREACH(epos::epos * node, axes)
	{
		node->setOpMode(epos::epos::OMD_HOMING_MODE);
	}

	// Do homing using preconfigured setup
	BOOST_FOREACH(epos::epos * node, axes)
	{
		node->startHoming();
	}

	// Loop until homing is finished
	bool finished;
	do {
		finished = true;
		BOOST_FOREACH(epos::epos * node, axes)
		{
			if (!node->isHomingFinished()) {
				finished = false;
			}
		}
	} while (!finished);

	// Hardcoded safety values
	// TODO: move to configuration file?

	for (std::size_t i = 0; i < axes.size(); ++i) {
		axes[i]->writeMinimalPositionLimit(kinematics::spkm::kinematic_parameters_spkm::lower_motor_pos_limits[i]-1);
		axes[i]->writeMaximalPositionLimit(kinematics::spkm::kinematic_parameters_spkm::upper_motor_pos_limits[i]+1);
	}

	// Just for testing if limits actually works
	//	axisA->writeMinimalPositionLimit(-100000);
	//	axisB->writeMinimalPositionLimit(-100000);
	//	axisC->writeMinimalPositionLimit(-100000);

	// Reset internal state of the motor positions
	for (int i = 0; i < number_of_servos; ++i) {
		current_motor_pos[i] = desired_motor_pos_old[i] = 0;
	}

	// Compute joints positions in the home position
	get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

	// Now the robot is synchronised
	controller_state_edp_buf.is_synchronised = true;
}

void effector::move_arm(const lib::c_buffer &instruction)
{
	try {
		switch (ecp_edp_cbuffer.variant)
		{
			case lib::spkm::POSE:
				switch (ecp_edp_cbuffer.pose_specification)
				{
					case lib::spkm::MOTOR:
						// Copy data directly from buffer
						for (int i = 0; i < number_of_servos; ++i) {
							desired_motor_pos_new[i] = ecp_edp_cbuffer.motor_pos[i];
							std::cout << "MOTOR[ " << i << "]: " << desired_motor_pos_new[i] << std::endl;
						}

						// Check desired joint values if they are absolute.
						if (is_synchronised()) {
							get_current_kinematic_model()->mp2i_transform(desired_motor_pos_new, desired_joints);
						}

						break;
					case lib::spkm::JOINT:
						// Copy data directly from buffer
						for (int i = 0; i < number_of_servos; ++i) {
							desired_joints[i] = ecp_edp_cbuffer.joint_pos[i];
							std::cout << "JOINT[ " << i << "]: " << desired_joints[i] << std::endl;
						}

						if (is_synchronised()) {
							// Transform desired joint to motors (and check motors/joints values).
							get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);
						} else {
							// Throw non-fatal error - this mode requires synchronization.
							BOOST_THROW_EXCEPTION(mrrocpp::kinematics::spkm::spkm_unsynchronized_error());
						}

						break;
					case lib::spkm::FRAME:
						std::cout<<"FRAME: [";
						for (unsigned int i = 0; i < 6; ++i) {
							std::cout<<ecp_edp_cbuffer.goal_pos[i]<<", ";
						}
						std::cout<<"]\n";

						if (is_synchronised()) {
							// Compute the desired homogeneous matrix on the base of received 6 variables related to angle-axis representation.
							desired_end_effector_frame.set_from_xyz_angle_axis(mrrocpp::lib::Xyz_Angle_Axis_vector(ecp_edp_cbuffer.goal_pos));
							//std::cout << desired_end_effector_frame<<std::endl;

							// Compute inverse kinematics for desired pose.
							get_current_kinematic_model()->inverse_kinematics_transform(desired_joints, current_joints, desired_end_effector_frame);

							// Transform joints to motors (and check motors/joints values).
							get_current_kinematic_model()->i2mp_transform(desired_motor_pos_new, desired_joints);
						} else {
							// Throw non-fatal error - this mode requires synchronization.
							BOOST_THROW_EXCEPTION(mrrocpp::kinematics::spkm::spkm_unsynchronized_error());
						}
						break;
					default:
						// Throw non-fatal error - invalid pose specification.
						BOOST_THROW_EXCEPTION(mrrocpp::kinematics::spkm::spkm_pose_specification_error());
						break;

				}

				// Note: at this point we assume, that desired_motor_pos_new holds a validated data.

				switch (ecp_edp_cbuffer.motion_variant)
				{
					case lib::epos::NON_SYNC_TRAPEZOIDAL:
						// Execute command
						for (std::size_t i = 0; i < axes.size(); ++i) {
							if (is_synchronised()) {
								std::cout << "MOTOR: moveAbsolute[" << i << "] ( " << desired_motor_pos_new[i] << ")"
										<< std::endl;
								if (!robot_test_mode) {
									axes[i]->writeProfileVelocity(Vdefault[i]);
									axes[i]->writeProfileAcceleration(Adefault[i]);
									axes[i]->writeProfileDeceleration(Ddefault[i]);
									axes[i]->moveAbsolute(desired_motor_pos_new[i]);
								} else {
									current_joints[i] = desired_joints[i];
									current_motor_pos[i] = desired_motor_pos_new[i];
								}
							} else {
								std::cout << "MOTOR: moveRelative[" << i << "] ( " << desired_motor_pos_new[i] << ")"
										<< std::endl;
								if (!robot_test_mode) {
									axes[i]->writeProfileVelocity(Vdefault[i]);
									axes[i]->writeProfileAcceleration(Adefault[i]);
									axes[i]->writeProfileDeceleration(Ddefault[i]);
									axes[i]->moveRelative(desired_motor_pos_new[i]);
								} else {
									current_joints[i] += desired_joints[i];
									current_motor_pos[i] += desired_motor_pos_new[i];
								}
							}
						}
						break;
					case lib::epos::SYNC_TRAPEZOIDAL: {
						Matrix <double, 6, 1> Delta, Vmax, Amax, Vnew, Anew, Dnew;

						for (int i = 0; i < 6; ++i) {
							Delta[i] = std::fabs(desired_motor_pos_new[i] - desired_motor_pos_old[i]);
							std::cout << "new - old[" << i << "]: " << desired_motor_pos_new[i] << " - "
									<< desired_motor_pos_old[i] << " = " << Delta[i] << std::endl;
							Vmax[i] = Vdefault[i];
							Amax[i] = Adefault[i];
						}

						// Calculate time of trapezoidal profile motion according to commanded acceleration and velocity limits
						double t = ppm <6> (Delta, Vmax, Amax, Vnew, Anew, Dnew);

						//					std::cerr <<
						//						"Delta:\n" << Delta << std::endl <<
						//						"Vmax:\n" << Vmax << std::endl <<
						//						"Amax:\n" << Amax << std::endl <<
						//						std::endl;

						if (t > 0) {
							std::cerr << "Vnew:\n" << Vnew << std::endl << "Anew:\n" << Anew << std::endl << "Dnew:\n"
									<< Dnew << std::endl << std::endl;

							if (!robot_test_mode) {
								// Setup motion parameters
								for (std::size_t i = 0; i < axes.size(); ++i) {
									if (Delta[i] != 0) {
										axes[i]->setOpMode(epos::epos::OMD_PROFILE_POSITION_MODE);
										axes[i]->writePositionProfileType(0); // Trapezoidal velocity profile
										axes[i]->writeProfileVelocity(Vnew[i]);
										axes[i]->writeProfileAcceleration(Anew[i]);
										axes[i]->writeProfileDeceleration(Dnew[i]);
										axes[i]->writeTargetPosition(desired_motor_pos_new[i]);
									}
								}
							}

							// Start motion
							for (std::size_t i = 0; i < axes.size(); ++i) {
								if (Delta[i] != 0) {
									if (is_synchronised()) {
										// Absolute motion
										if (!robot_test_mode) {
											axes[i]->writeControlword(0x3f);
										} else {
											current_joints[i] = desired_joints[i];
											current_motor_pos[i] = desired_motor_pos_new[i];
										}
									} else {
										// Relative motion
										if (!robot_test_mode) {
											axes[i]->writeControlword(0x005f);
										} else {
											current_joints[i] += desired_joints[i];
											current_motor_pos[i] += desired_motor_pos_new[i];
										}
									}
								}
							}
						}
					}
						break;
					default:
						// Throw non-fatal error - motion type not supported.
						BOOST_THROW_EXCEPTION(mrrocpp::kinematics::spkm::spkm_motion_type_error());
						break;
				}
				break;
			case lib::spkm::QUICKSTOP:
				if (!robot_test_mode) {
					// Execute command
					BOOST_FOREACH(epos::epos * node, axes)
					{
						// Brake with Quickstop command
						node->changeEPOSstate(epos::epos::QUICKSTOP);
					}
				}
				break;
			case lib::spkm::CLEAR_FAULT:
				BOOST_FOREACH(epos::epos * node, axes)
				{
					node->printEPOSstate();

					// Check if in a FAULT state
					if (node->checkEPOSstate() == 11) {
						epos::UNSIGNED8 errNum = node->readNumberOfErrors();
						std::cerr << "readNumberOfErrors() = " << (int) errNum << std::endl;
						for (epos::UNSIGNED8 i = 1; i <= errNum; ++i) {

							epos::UNSIGNED32 errCode = node->readErrorHistory(i);

							std::cerr << epos::epos::ErrorCodeMessage(errCode) << std::endl;
						}
						if (errNum > 0) {
							node->clearNumberOfErrors();
						}
						node->changeEPOSstate(epos::epos::FAULT_RESET);
					}

					// Change to the operational mode
					node->reset();
				}
			default:
				break;
		}

		for (int i = 0; i < 6; ++i) {
			std::cout << "OLD     MOTOR[" << i << "]: " << desired_motor_pos_old[i] << std::endl;
			std::cout << "CURRENT MOTOR[" << i << "]: " << current_motor_pos[i] << std::endl;
			std::cout << "CURRENT JOINT[" << i << "]: " << current_joints[i] << std::endl;
		}

		// Hold the issued command
		desired_motor_pos_old = desired_motor_pos_new;
	} catch (mrrocpp::lib::exception::mrrocpp_non_fatal_error e_) {
		std::cout<<boost::current_exception_diagnostic_information()<<std::endl;
	}
}

void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	// we do not check the arm position when only lib::SET is set
	if (instruction.instruction_type != lib::SET) {
		switch (instruction.get_arm_type)
		{
			case lib::MOTOR:
				msg->message("EDP get_arm_position MOTOR");
				for (std::size_t i = 0; i < axes.size(); ++i) {
					if (robot_test_mode) {
						edp_ecp_rbuffer.epos_controller[i].position = current_motor_pos[i];
						edp_ecp_rbuffer.epos_controller[i].current = 0;
						edp_ecp_rbuffer.epos_controller[i].motion_in_progress = false;
					} else {
						current_motor_pos[i] = axes[i]->readActualPosition();
						edp_ecp_rbuffer.epos_controller[i].position = current_motor_pos[i];
						edp_ecp_rbuffer.epos_controller[i].current = axes[i]->readActualCurrent();
						edp_ecp_rbuffer.epos_controller[i].motion_in_progress = !axes[i]->isTargetReached();
					}
				}
				break;
			case lib::JOINT:
				msg->message("EDP get_arm_position JOINT");

				// Read actual values from hardware
				if (!robot_test_mode) {
					for (std::size_t i = 0; i < axes.size(); ++i) {
						current_motor_pos[i] = axes[i]->readActualPosition();
					}
				}

				// Do the calculation
				get_current_kinematic_model()->mp2i_transform(current_motor_pos, current_joints);

				// Fill the values into a buffer
				for (int i = 0; i < number_of_servos; ++i) {
					edp_ecp_rbuffer. epos_controller[i].position = current_joints[i];
				}
				break;
			case lib::FRAME: {
				msg->message("EDP get_arm_position FRAME");

				lib::Homog_matrix tmp_frame;

				tmp_frame.get_frame_tab(edp_ecp_rbuffer.current_frame);
			}
				break;
			default:
				break;

		}
	}

	reply.servo_step = step_counter;
}
/*--------------------------------------------------------------------------*/

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::spkm::kinematic_model_spkm());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

/*--------------------------------------------------------------------------*/
/*                           Utility routines                               */
/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	//vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);
}

void effector::instruction_deserialization()
{
	memcpy(&ecp_edp_cbuffer, instruction.arm.serialized_command, sizeof(ecp_edp_cbuffer));
}

void effector::reply_serialization(void)
{
	memcpy(reply.arm.serialized_reply, &edp_ecp_rbuffer, sizeof(edp_ecp_rbuffer));
	assert(sizeof(reply.arm.serialized_reply) >= sizeof(edp_ecp_rbuffer));
}

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	manip_effector::single_thread_master_order(nm_task, nm_tryb);
}

} // namespace spkm


namespace common {

// Stworzenie obiektu edp_irp6m_effector.
effector* return_created_efector(lib::configurator &_config)
{
	return new spkm::effector(_config);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
