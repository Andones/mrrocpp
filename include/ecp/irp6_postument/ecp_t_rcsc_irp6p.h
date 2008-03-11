#if !defined(_ECP_T_RCSC_IRP6P_H)
#define _ECP_T_RCSC_IRP6P_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp/common/ecp_st_go.h"

class ecp_task_rcsc_irp6p: public ecp_task
{
protected:
    //generatory
    ecp_generator_t* gt;
    ecp_tff_nose_run_generator* nrg;
    ecp_tff_rubik_grab_generator* rgg;
    ecp_tff_gripper_approach_generator* gag;
    ecp_tff_rubik_face_rotate_generator* rfrg;
    ecp_teach_in_generator* tig;
    ecp_smooth_generator* sg;
    bias_edp_force_generator* befg;
    weight_meassure_generator* wmg;
    //podzadania
    ecp_sub_task_gripper_opening* go_st;

public:
    // KONSTRUKTORY
    ecp_task_rcsc_irp6p(configurator &_config);
    ~ecp_task_rcsc_irp6p();

    // methods for ECP template to redefine in concrete classes
    void task_initialization(void);
    void main_task_algorithm(void);

};

#endif
