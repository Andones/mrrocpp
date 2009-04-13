#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_jarosz.h"
#include "ecp/irp6_on_track/ecp_t_pt_irp6ot.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


// KONSTRUKTORY
pteach::pteach(configurator &_config) : base(_config)
{
    cg = NULL;
    orc = NULL;
}

// methods for ECP template to redefine in concrete classes
void pteach::task_initialization(void)
{
    ecp_m_robot = new ecp_irp6_on_track_robot (*this);

    cg = new common::generator::calibration (*this, 10);
    // Warunek, ktorego spelnienie umozliwia realizacje ruchu do nastepnej nauczonej pozycji
    orc = new common::operator_reaction_condition (*this);

    sr_ecp_msg->message("ECP loaded");
}


void pteach::main_task_algorithm(void)
{
	if (operator_reaction("Teach in?"))
    {
        cg->flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
        cg->teach ( MOTOR, "Teach-in the trajectory\n");
        if ( operator_reaction ("Save trajectory?") )
        {
            cg->save_file (MOTOR);
        }
    }
    else
        if ( operator_reaction ("Load trajectory?") )
        {
            cg->load_file_from_ui ();
        }

    if ( !operator_reaction ("Start motion?") )
    {
        // Informacja dla MP o zakonczeniu zadania uzytkownika
        ecp_termination_notice ();
        return;
    }

    // Odtwarzanie nauczonej lub wczytanej trajektorii zaczynamy od jej poczatku
    cg->initiate_pose_list();
    // Dlugosc listy = liczba pozycji do odtworzenia
    pll = cg->pose_list_length();
    // Na razie lista pozycji odczytanych we wspolrzednych kartezjanskich jest pusta
    orc->flush_supplementary_list();
    for (i=0; i< pll; i++)
    { // Wewnetrzna petla wykonuje sie pll razy (tyle pozycji nauczono)
        cg->Move();
        orc->Move();
        // oraz oczekiwanie na zezwolenie na kolejny ruch
    } // end: for

    // Czy zapamietac dane?
    if ( operator_reaction ("Save calibration data?") )
    {
        ecp_save_extended_file (*cg, *orc, *this);
    }

    // Informacja dla MP o zakonczeniu zadania uzytkownika
    ecp_termination_notice ();
}

}
} // namespace irp6ot

namespace common {
namespace task {

base* return_created_ecp_task (configurator &_config)
{
	return new irp6ot::task::pteach(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

