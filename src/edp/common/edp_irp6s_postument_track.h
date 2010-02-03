// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_IRP6S_P_T_H
#define __EDP_IRP6S_P_T_H

#include <semaphore.h>

#include "edp/common/edp_e_manip.h"
#include "edp/common/edp_force_sensor.h"

namespace mrrocpp {
namespace edp {
namespace common {

// base class for EDP robots with manipulators postument and on_track


/************************ edp_irp6s_effector ****************************/
class irp6s_postument_track_effector: public manip_effector
{

protected:


    // czy chwytak wlaczony ?
    short is_gripper_active;

	// numer serwo chwytaka
	short gripper_servo_nr;

    void compute_frame(const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (koncowka: FRAME)

    // Przeksztalcenie definicji koncowki z postaci
    // FRAME do postaci XYZ_EULER_ZYZ
    // oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych REPLY_BUFFER
  //  void arm_frame_2_pose_force_torque_at_frame (bool* = NULL);
    // Przeksztalcenie definicji koncowki z postaci
    // FRAME do postaci XYZ_EULER_ZYZ
    // oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych REPLY_BUFFER

public:

	irp6s_postument_track_effector (lib::configurator &_config, lib::robot_name_t l_robot_name );       // konstruktor

    int force_tryb;

    void pose_force_torque_at_frame_move (const lib::c_buffer &instruction);

    void iterate_macrostep(const lib::JointArray begining_joints,
       		const lib::Homog_matrix begining_end_effector_frame,
       		const lib::c_buffer &instruction, const lib::Xyz_Angle_Axis_vector base_pos_xyz_rot_xyz_vector);
    void compute_base_pos_xyz_rot_xyz_vector(const lib::JointArray begining_joints,
    		const lib::Homog_matrix begining_end_effector_frame,
    		const lib::c_buffer &instruction, lib::Xyz_Angle_Axis_vector& base_pos_xyz_rot_xyz_vector);

    bool servo_joints_and_frame_actualization_and_upload(void);// by Y

    void create_threads ();

    void set_rmodel (lib::c_buffer &);
    void get_rmodel (lib::c_buffer &);
    void move_arm (lib::c_buffer &);
    void get_arm_position(bool, lib::c_buffer &);
	void master_order(MT_ORDER nm_task, int nm_tryb);
};

/************************ edp_irp6s_effector ****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp


#endif
