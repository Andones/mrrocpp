// -------------------------------------------------------------------------
// Plik:			ecp_mp_t_fsautomat.h
// Autor:		Marek Kisiel
// Data:			2008
// -------------------------------------------------------------------------

#ifndef __ECP_MP_T_FSAUTOMAT_H
#define __ECP_MP_T_FSAUTOMAT_H

namespace mrrocpp {
namespace ecp_mp {
namespace task {

enum POURING_GRIPPER_OP
{
	RCSC_GO_VAR_1, RCSC_GO_VAR_2
};

enum POURING_PHASES
{
	POURING_PHASE_1, POURING_PHASE_2
};


const std::string ECP_GEN_SPEAK = "ECP_GEN_SPEAK";
const std::string ECP_TOOL_CHANGE_GENERATOR = "ECP_TOOL_CHANGE_GENERATOR";

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif// -------------------------------------------------------------------------
