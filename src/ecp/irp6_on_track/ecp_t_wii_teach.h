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
        wiimote_t lastButtons;
        wiimote_t buttonsPressed;

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
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif
