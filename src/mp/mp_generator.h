#ifndef MP_GENERATOR_H_
#define MP_GENERATOR_H_

#include "mp/mp_robot.h"
#include "ecp_mp/ecp_mp_generator.h"

namespace mrrocpp {
namespace mp {
namespace generator {

class generator : public ecp_mp::generator::generator
{
	private:

		//! Kopiuje dane z robotow do generatora
		void copy_data(const common::robots_t & _robot_m);

		//! Kopiuje polecenie stworzone w generatorze do robotow
		void copy_generator_command (const common::robots_t & _robot_m);

		// Zadanie, któremu podlega generator
		task::task& mp_t;

	protected:

		int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)

	public:

		// Funkcja ruchu
		void Move (void);

		/*!
		 * okresla czy przed next step Move ma sie zawieszac w oczekwianiu na puls z ECP;
		 * wykorzystywane przy luznej i sporadycznej wspolpracy robotow.
		 */
		bool wait_for_ECP_pulse;

		//! mapa wszystkich robotow
		common::robots_t robot_m;

		//! Konstruktor
		generator(task::task& _mp_task);

		//! Klasa obslugi bledow generatora na poziomie MP
		class MP_error
		{
			public:
				const lib::error_class_t error_class;
				const uint64_t error_no;

				MP_error(lib::error_class_t err0, uint64_t err1) :
					error_class(err0), error_no(err1)
				{
				}
		};
};

} // namespace common
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATOR_H_*/
