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

    pthread_t force_tid;
    pthread_t edp_vsp_tid;

    void arm_abs_xyz_eul_zyz_2_frame (const double *p);
    // Przeksztalcenie definicji koncowki z postaci
    // XYZ_EULER_ZYZ wyrazonej bezwzglednie do postaci
    // FRAME oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych TRANSFORMATORa

    void arm_frame_2_xyz_eul_zyz ();

    // Przeksztalcenie definicji koncowki z postaci
    // FRAME do postaci XYZ_EULER_ZYZ
    // oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych REPLY_BUFFER
  //  void arm_frame_2_pose_force_torque_at_frame (bool* = NULL);
    // Przeksztalcenie definicji koncowki z postaci
    // FRAME do postaci XYZ_EULER_ZYZ
    // oraz przepisanie wyniku przeksztalcenia do
    // wewnetrznych struktur danych REPLY_BUFFER

    double global_kartez_force_msr[7], prevoius_global_kartez_force_msr[7]; // sila we wspolrzednych kartezjankich    XXXXX
    // 	i	 odczytana bezposrednio z czujnika - zestaw globalny dla procesu EDP

    pthread_mutex_t force_mutex;	// mutex do sily   XXXXXX

public:

	sem_t force_master_sem; //!< semafor dostepu do nowej wiadomosci dla vsp

    irp6s_postument_track_effector (lib::configurator &_config, lib::robot_name_t l_robot_name );       // konstruktor

    int force_tryb;

    // TODO: rename ruch pozycyjno silowo dla staqlej orientacji i kierukow liniowych
    virtual void pose_force_linear_move (lib::c_buffer &instruction);

    void pose_force_torque_at_frame_move (lib::c_buffer &instruction);

    void servo_joints_and_frame_actualization_and_upload(void);// by Y

    void force_msr_upload(const double *new_value);// by Y wgranie globalnego zestawu danych

    // value to 7 elemetnowa tablica short
    // tryb - 0 zestaw kartezjanski, 1 zestaw nieprzetworzony

    lib::Homog_matrix return_current_frame(TRANSLATION_ENUM translation_mode);// by Y przepisanie z zestawu globalnego na lokalny edp_force

    // wyznaczenie polozenia lokalnego i globalnego transformera
    // przepisanie lokalnego zestawu lokalnego edp_servo na globalny (chronione mutexem)
    void master_joints_and_frame_download(void);// by Y przepisanie z zestawu globalnego na lokalny dla edp_master

    lib::frame_tab force_current_end_effector_frame;// by Y dla watku EDP_FORCE

    void force_msr_download(double *new_value, double *old_value);// by Y odczytanie globalnego zestawu danych

    void create_threads ();

    void set_rmodel (lib::c_buffer &);
    void get_rmodel (lib::c_buffer &);
    void move_arm (lib::c_buffer &);
    void get_arm_position(bool, lib::c_buffer &);

    virtual servo_buffer* return_created_servo_buffer ();
};
/************************ edp_irp6s_effector ****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp


#endif
