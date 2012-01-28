/*!
 * @file mp_t_single_agent_demo.cpp
 *
 * @date Jan 20, 2012
 * @author tkornuta
 */

#include <fstream>

#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>

#include "mp_t_single_agent_plan.h"

#include "base/lib/configurator.h"
#include "base/ecp_mp/ecp_ui_msg.h"

#include "../swarmitfix_plan/plan_iface.h"
#include "plan.hxx"

#include "robot/shead/kinematic_model_shead.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix::single_agent_demo(_config);
}

namespace swarmitfix {

void single_agent_demo::executeCommandItem(const Plan::PkmType::ItemType & pkmCmd)
{
	// Make sure that there are only Xyz-Euler-Zyz coordinates.
	assert(pkmCmd.pkmToWrist().present() == false);

	// Only n*20 index allowed for base commands.
	assert((pkmCmd.ind() + 100) % 20 == 0);

	// Check if command is allowed in a given state.
	const int last_state = agent_last_state[pkmCmd.agent()-1];
	const int new_state = (pkmCmd.ind() + 100) % 100;

	switch(last_state) {
		case 0:
			switch(new_state) {
				case 20:
				case 80:
				case 0:
					// SUPPORT->PRE/POST pose.
					break;
				default:
					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "Only PRE/POST pose allowed from SUPPORT");
					return;
			}
			break;
		case 20:
		case 80:
			// Everything is allowed from PRE/POST pose.
			break;
		case 40:
		case 50:
		case 60:
			switch(new_state) {
				case 20:
				case 40:
				case 60:
				case 80:
					// NEUTRAL->PRE/POST/NEUTRAL pose.
					break;
				default:
					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "Only PRE/POST pose allowed from NEUTRAL");
					return;
			}
			break;
		case -1:
			switch(new_state) {
				case 20:
				case 80:
					// UNKNOWN->PRE/POST pose.
					break;
				default:
					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "Only PRE/POST pose allowed from UNKNOWN");
					return;
			}
			break;
		default:
			// This should not happend.
			assert(0);
			break;
	}

	// PKM pose
	lib::Xyz_Euler_Zyz_vector goal(
			pkmCmd.Xyz_Euler_Zyz()->x(),
			pkmCmd.Xyz_Euler_Zyz()->y(),
			pkmCmd.Xyz_Euler_Zyz()->z(),
			pkmCmd.Xyz_Euler_Zyz()->alpha(),
			pkmCmd.Xyz_Euler_Zyz()->beta(),
			pkmCmd.Xyz_Euler_Zyz()->gamma()
			);

	std::cout << "Xyz_Euler_Zyz: " <<
			pkmCmd.Xyz_Euler_Zyz()->x() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->y() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->z() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->alpha() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->beta() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->gamma() << std::endl;

	// Head pose.
	double head_pose = pkmCmd.beta7();

	// Adjust head offset.
	if(pkmCmd.beta7() < kinematics::shead::model::getLowerJointLimit()) {
		head_pose += M_PI/3;
	}
	if(pkmCmd.beta7() > kinematics::shead::model::getUpperJointLimit()) {
		head_pose -= M_PI/3;
	}

	// Check if above is enough.
	assert(head_pose >= kinematics::shead::model::getLowerJointLimit());
	assert(head_pose <= kinematics::shead::model::getUpperJointLimit());

	// Check if robot name match with item.
	lib::robot_name_t spkm_robot_name;
	lib::robot_name_t shead_robot_name;

	if (pkmCmd.agent() == 1) {
		spkm_robot_name = lib::spkm1::ROBOT_NAME;
		shead_robot_name = lib::shead1::ROBOT_NAME;
	} else if (pkmCmd.agent() == 2) {
		spkm_robot_name = lib::spkm2::ROBOT_NAME;
		shead_robot_name = lib::shead2::ROBOT_NAME;
	} else {
		assert(0);
	}

	// PRE-head command;
	if(is_robot_activated(shead_robot_name)) {

		// Disable solidification and vacuum.
		shead_solidify(shead_robot_name, false);
		shead_vacuum(shead_robot_name, false);

		switch((pkmCmd.ind() + 100) % 100) {
			case 0:
			case 40:
			case 80:
				move_shead_joints(shead_robot_name, head_pose);
				break;
			default:
				break;
		}
	}

	// Execute PKM command.
	if(is_robot_activated(spkm_robot_name)) move_spkm_external(spkm_robot_name, lib::epos::SYNC_TRAPEZOIDAL, goal);

	// POST-head command;
	if(is_robot_activated(shead_robot_name)) {
		switch((pkmCmd.ind() + 100) % 100) {
			case 0:
				// Apply vacuum...
				shead_vacuum(shead_robot_name, false); // FIXME: turn on the vacuum.
				// ...wait a while...
				boost::this_thread::sleep(boost::posix_time::milliseconds(100));
				// ...apply solidification...
				shead_solidify(shead_robot_name, false); // FIXME: turn on the solidification.
				// ...and brake.
				spkm_brake(spkm_robot_name);
				break;
			default:
				break;
		}
	}

	// Update state of the agent.
	agent_last_state[pkmCmd.agent()-1] = (pkmCmd.ind() + 100) % 100;
}

void single_agent_demo::executeCommandItem(const Plan::MbaseType::ItemType & smbCmd, int dir)
{
	// TODO: Only single-item actions are supported at this time.
	assert(smbCmd.actions().item().size() == 1);

	// Only x50 index allowed for base commands.
	assert((smbCmd.ind() + 100) % 100 == 50);

	// Check if rotation is allowed in the current state.
	switch(agent_last_state[smbCmd.agent()-1] % 100) {
		case 40:
		case 50:
		case 60:
			// PKM is in neutral pose.
			break;
		default:
			sr_ecp_msg->message(lib::NON_FATAL_ERROR, "Rotation prohibited in non-neutral pose");
			return;
	}

	// Check if robot name match with item.
	lib::robot_name_t smb_robot_name;

	if (smbCmd.agent() == 1) {
		smb_robot_name = lib::smb1::ROBOT_NAME;
	} else if (smbCmd.agent() == 2) {
		smb_robot_name = lib::smb2::ROBOT_NAME;
	} else {
		assert(0);
	}

	// Check if robot name match with item.
	if(is_robot_activated(smb_robot_name)) {
		// Execute command.
		smb_rotate_external(smb_robot_name, 0, smbCmd.pkmTheta());
	}

	// Update state of the agent.
	agent_last_state[smbCmd.agent()-1] = (smbCmd.ind() + 100) % 100;
}

single_agent_demo::single_agent_demo(lib::configurator &config_) :
		demo_base(config_)
{
	// Read plan from file.
	p = readPlanFromFile(config.value<std::string>("planpath"));

	// Initialize state of the agents.
	agent_last_state[0] = -1;
	agent_last_state[1] = -1;
}

void single_agent_demo::create_robots()
{
	// Activate robots (depending on the configuration settings).
	// SMB.

	// SPKM.
	ACTIVATE_MP_ROBOT(spkm1)
	ACTIVATE_MP_ROBOT(spkm2)

	// SHEAD.
	ACTIVATE_MP_ROBOT(shead1)
	ACTIVATE_MP_ROBOT(shead2)

	// SMB.
	ACTIVATE_MP_ROBOT(smb1)
	ACTIVATE_MP_ROBOT(smb2)

	// Activate the SBENCH robot.
	ACTIVATE_MP_ROBOT(sbench)
}

void single_agent_demo::save_plan(const Plan & p)
{
	std::cout << "Save to file." << std::endl;
	std::string savepath = "foo.xml"; // config.value<std::string>(planner::planpath);
	std::ofstream ofs (savepath.c_str());
	plan(ofs, p);
}

lib::UI_TO_ECP_REPLY single_agent_demo::step_mode(Mbase::ItemType & item)
{
	// Create the text representation.
	std::ostringstream ostr;
	{
		boost::archive::text_oarchive oa(ostr);
		xml_schema::ostream<boost::archive::text_oarchive> os(oa);

		// serialize data
		os << item;
	}

	// Request
	lib::ECP_message ecp_to_ui_msg;

	// Setup plan item
	ecp_to_ui_msg.ecp_message = lib::PLAN_STEP_MODE;
	ecp_to_ui_msg.plan_item_type = lib::MBASE_AND_BENCH;
	ecp_to_ui_msg.plan_item_string = ostr.str();

	// Reply
	lib::UI_reply ui_to_ecp_rep;

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {

		uint64_t e = errno;
		perror("ecp operator_reaction(): Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(lib::exception::system_error());
	}

	if(ui_to_ecp_rep.reply == lib::PLAN_EXEC) {
		std::istringstream istr(ui_to_ecp_rep.plan_item_string);
		boost::archive::text_iarchive ia(istr);
		xml_schema::istream<boost::archive::text_iarchive> is (ia);

		// deserialize data
		item = Mbase::ItemType(is);
	}

	return ui_to_ecp_rep.reply;
}

lib::UI_TO_ECP_REPLY single_agent_demo::step_mode(Pkm::ItemType & item)
{
	// create archive
	std::ostringstream ostr;
	{
		boost::archive::text_oarchive oa(ostr);
		xml_schema::ostream<boost::archive::text_oarchive> os(oa);

		// serialize data
		os << item;
	}

	// Request
	lib::ECP_message ecp_to_ui_msg;

	// Setup plan item
	ecp_to_ui_msg.ecp_message = lib::PLAN_STEP_MODE;
	ecp_to_ui_msg.plan_item_type = lib::PKM_AND_HEAD;
	ecp_to_ui_msg.plan_item_string = ostr.str();

	// Reply
	lib::UI_reply ui_to_ecp_rep;

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {

		uint64_t e = errno;
		perror("ecp operator_reaction(): Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(lib::exception::system_error());
	}

	if(ui_to_ecp_rep.reply == lib::PLAN_EXEC) {
		std::istringstream istr(ui_to_ecp_rep.plan_item_string);
		boost::archive::text_iarchive ia(istr);
		xml_schema::istream<boost::archive::text_iarchive> is (ia);

		// deserialize data
		item = Pkm::ItemType(is);
	}

	return ui_to_ecp_rep.reply;
}

#define EXECUTE_PLAN_IN_STEP_MODE	1

void single_agent_demo::main_task_algorithm(void)
{
	/////////////////////////////////////////////////////////////////////////////////
	using namespace mrrocpp::lib::sbench;
	/////////////////////////////////////////////////////////////////////////////////

	// FIXME: move this to config file.
	const bool execute_plan_in_step_mode = true;

	// Time index counter
	int indMin = 99999999, indMax = 0;

	// Setup index counter at the beginning of the plan
	BOOST_FOREACH(const Plan::PkmType::ItemType & it, p->pkm().item()) {
		if(indMin > it.ind()) indMin = it.ind();
		if(indMax < it.ind()) indMax = it.ind();
	}
	BOOST_FOREACH(const Plan::MbaseType::ItemType & it, p->mbase().item()) {
		if(indMin > it.ind()) indMin = it.ind();
		if(indMax < it.ind()) indMax = it.ind();
	}

	// Make sure that modulo 100 arithmetics operate on positives.
	assert(indMin > -99);

	for (int ind = indMin, dir = 0; true; ind += dir) {

		if(ind < indMin) ind = indMin;
		if(ind > indMax) ind = indMax;

		std::cout << "plan index = " << ind << std::endl;

		// Diagnostic timestamp
		boost::system_time start_timestamp;

		// Plan iterators
		const Plan::PkmType::ItemSequence::iterator pkm_it = StateAtInd(ind, p->pkm().item());
		const Plan::MbaseType::ItemSequence::iterator smb_it = StateAtInd(ind, p->mbase().item());

		// Current state
		State * currentActionState;

		bool record_timestamp = false;

		// Execute matching command item
		if(pkm_it != p->pkm().item().end()) {

			if(execute_plan_in_step_mode) {
				switch(step_mode(*pkm_it)) {
					case lib::PLAN_PREV:
						dir = -1;
						break;
					case lib::PLAN_NEXT:
						dir = +1;
						break;
					case lib::PLAN_EXEC:
						currentActionState = (State *) &(*pkm_it);
						start_timestamp = boost::get_system_time();
						executeCommandItem(*pkm_it);
						record_timestamp = true;
						dir = 0;
						break;
					case lib::PLAN_SAVE:
						save_plan(*p);
						dir = 0;
						break;
					default:
						break;
				}
			} else {
				currentActionState = (State *) &(*pkm_it);
				start_timestamp = boost::get_system_time();
				executeCommandItem(*pkm_it);
				record_timestamp = true;
				dir = 1;
			}

		} else if(smb_it != p->mbase().item().end()) {

			if(execute_plan_in_step_mode) {
				switch(step_mode(*smb_it)) {
					case lib::PLAN_PREV:
						dir = -1;
						break;
					case lib::PLAN_NEXT:
						dir = +1;
						break;
					case lib::PLAN_EXEC:
						currentActionState = (State *) &(*smb_it);
						start_timestamp = boost::get_system_time();
						executeCommandItem(*smb_it, dir);
						record_timestamp = true;
						dir = 0;
						break;
					case lib::PLAN_SAVE:
						save_plan(*p);
						dir = 0;
						break;
					default:
						break;
				}
			} else {
				currentActionState = (State *) &(*smb_it);
				start_timestamp = boost::get_system_time();
				executeCommandItem(*smb_it, dir);
				record_timestamp = true;
				dir = 1;
			}
		} else {
			continue;
		}


		// Diagnostic timestamp
		if (record_timestamp) {
			// Get the stop timestamp
			boost::system_time stop_timestamp = boost::get_system_time();

			// Calculate command duration
			boost::posix_time::time_duration td = stop_timestamp - start_timestamp;

			std::cout << "Command duration in [ms] is " << td.total_milliseconds() << std::endl;
			currentActionState->state_reached_in_time().set(td.total_milliseconds()/1000.0);

//			// Wait for trigger
//			while(!(ui_pulse.isFresh() && ui_pulse.Get() == MP_TRIGGER)) {
//				// TODO: handle PAUSE/RESUME/STOP commands as well
//				if(ui_pulse.isFresh()) ui_pulse.markAsUsed();
//				ReceiveSingleMessage(true);
//			}
//
//			ui_pulse.markAsUsed();

		}

		// If all iterators are at the end...
		if(ind == indMax && dir > 0)
		{
			// ...then finish
			break;
		}
	}

	// Serialize to a file.
	//
	{
		std::cout << "Serialize to a file." << std::endl;
		std::ofstream ofs ("result.xml");
		plan(ofs, *p);
	}

	sr_ecp_msg->message("END");


	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	return;

}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
