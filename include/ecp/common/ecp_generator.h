/*!
 * \file ecp_generator.h
 * \brief ECP generators base class.
 * - class declaration
 * \author twiniarsk
 * \date 17.03.2008
 */

#if !defined(_ECP_GENERATOR_H)
#define _ECP_GENERATOR_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_robot.h"
#include "ecp_mp/ecp_mp_generator.h"



class ecp_generator : public ecp_mp::generator::ecp_mp_generator {

	protected:
		ecp_task& ecp_t;

	public:
	    // Zlecenie ruchu dla EDP
  		void Move(void);
  		virtual void execute_motion (void);
	
		bool communicate_with_mp_in_move;
		bool communicate_with_edp;
		bool copy_edp_buffers_in_move;
		ecp_robot* the_robot;

		ecp_generator(ecp_task& _ecp_task);

		virtual ~ecp_generator();
		

		bool is_EDP_error (ecp_robot& the_robot) const;

		class ECP_error {  // Klasa obslugi bledow generatora
			public:
				const uint64_t error_class;
				const uint64_t error_no;
				edp_error error;

				ECP_error ( uint64_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
		}; // end: class ECP_error

};

#endif /* _ECP_GENERATOR_H */
