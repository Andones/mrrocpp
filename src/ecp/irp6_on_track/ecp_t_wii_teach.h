#if !defined(_ECP_T_WII_TEACH_H)
#define _ECP_T_WII_TEACH_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/common/ecp_g_smooth2.h"
#include "ecp/irp6_on_track/ecp_g_wii_teach.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

/**
 * @author jkurylo
 */
class wii_teach: public common::task::task
{
    protected:
	//Generator ruchu
        common::generator::smooth2* sg;
        irp6ot::generator::wii_teach* wg;
        lib::sensor_image_t::sensor_union_t::wiimote_t lastButtons;
        lib::sensor_image_t::sensor_union_t::wiimote_t buttonsPressed;

        class n;
        class n
        {
            public:
                n* next;
                n* prev;
                int id;
                double position[8];

                n() : next(NULL), prev(NULL) {}

        };

        typedef n node;

        struct
        {
            node* head;
            node* tail;
            int count;
            node* current;
            int position;
        } trajectory;

        void updateButtonsPressed();

    public:
	/**
	 * Tworzy obiekt zadania
	 * @param _config konfigurator
	 * @author jedrzej
	 */
	wii_teach(lib::configurator &_config);

	/**
	 * Realizuje zadanie
	 * @author jkurylo
	 */
	void main_task_algorithm(void);

        void print_trajectory(void);

        void move_to_current(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif
