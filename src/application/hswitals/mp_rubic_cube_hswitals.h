#if !defined(__MP_T_HSWITALS_GENERATORE_H)
#define __MP_T_HSWITALS_GENERATORE_H

#include "base/mp/mp_task.h"

namespace mrrocpp {
namespace mp {
namespace task {

#define MULTI 8
#define MULTI_RD 2

class rubik_cube : public task
{
public:
    double f_inertia;
    double t_inertia;
    double f_reciprocal_damping;
    double t_reciprocal_damping;

	/// utworzenie robotow
    void create_robots(void);
    void configure_edp_force_sensor(void);

	// konstruktor
    rubik_cube(lib::configurator &_config);

    ~rubik_cube();

    void approach_cube(void);
    void original_task(void);
    void modified_task(void);
    void departure_cube(void);

	// methods for mp template
	void main_task_algorithm(void);
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
