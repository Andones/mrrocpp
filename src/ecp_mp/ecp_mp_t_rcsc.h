// -------------------------------------------------------------------------
// Proces:
// Plik:			ecp_mp_t_rcsc.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Ogolna struktura obrazow czujnika
// Autor:		yoyek
// Data:		2007
// -------------------------------------------------------------------------

#ifndef __ECP_MP_T_RCSC_H
#define __ECP_MP_T_RCSC_H

namespace mrrocpp {
namespace ecp_mp {
namespace task {

enum RCSC_ECP_STATES {ECP_GEN_TRANSPARENT, ECP_GEN_TFF_NOSE_RUN, ECP_GEN_TEACH_IN,
                      ECP_GEN_SMOOTH, ECP_GEN_SMOOTH2,
                      ECP_GEN_TFF_RUBIK_GRAB, ECP_GEN_TFF_RUBIK_FACE_ROTATE, ECP_GEN_TFF_GRIPPER_APPROACH,
                      RCSC_GRIPPER_OPENING, ECP_GEN_SPEAK,   ECP_GEN_BIAS_EDP_FORCE,
                      ECP_WEIGHT_MEASURE_GENERATOR
                     };

enum RCSC_GRIPPER_OP {RCSC_GO_VAR_1, RCSC_GO_VAR_2};

enum RCSC_TURN_ANGLES {RCSC_CCL_90, RCSC_CL_0, RCSC_CL_90, RCSC_CL_180};

enum RCSC_RUBIK_GRAB_PHASES { RCSC_RG_FROM_OPEARTOR_PHASE_1, RCSC_RG_FROM_OPEARTOR_PHASE_2,
                              RCSC_RG_FCHANGE_PHASE_1, RCSC_RG_FCHANGE_PHASE_2, RCSC_RG_FCHANGE_PHASE_3, RCSC_RG_FCHANGE_PHASE_4
                            };

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif
