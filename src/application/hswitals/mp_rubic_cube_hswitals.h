// -------------------------------------------------------------------------
//                            mp_task_rcsc.h
// Definicje struktur danych i metod dla procesow MP - zadanie ukladania kostki Rubika
//  wersja z generatorami uruchaminami na poziomie ECP
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_RCSC_H)
#define __MP_TASK_RCSC_H

#include "application/rcsc/ecp_mp_t_rcsc.h"

namespace mrrocpp {
namespace mp {
namespace task {

class rubik_cube : public task
{
public:
	/// utworzenie robotow
    void create_robots(void);
    void configure_edp_force_sensor(void);

	// konstruktor
    rubik_cube(lib::configurator &_config);

    ~rubik_cube();

	// methods for mp template
	void main_task_algorithm(void);
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
