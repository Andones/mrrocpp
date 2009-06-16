// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie ukladania kostki Rubika
// 
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_RC_H)
#define __MP_TASK_RC_H

#include "mp/mp.h"
#include <list>

#include "mp/CubeState.h"
#include "mp/SingleManipulation.h"

namespace mrrocpp {
namespace mp {
namespace task {

class rubik_cube_solver : public task  
{
protected:
// sekwencja (lista) manipulacji

// stan kostki
// kolory scian patrzac przez os ramienia tracka (od kolumny), w plaszczynie ziemi
	common::CubeState* cube_state;
 
  
    // odczyt konfiguracji manipulacji
	char* cube_initial_state;
 
 
public:

	// stl'owa lista manipulacji
	std::list<common::SingleManipulation> manipulation_list;

	void initiate (common::CUBE_COLOR up_is, common::CUBE_COLOR down_is, common::CUBE_COLOR front_is, 
			common::CUBE_COLOR rear_is, common::CUBE_COLOR left_is, common::CUBE_COLOR right_is);

    // konstruktor
    rubik_cube_solver(lib::configurator &_config);
	
    ~rubik_cube_solver();

	
	// MANIPULACJA
	// manipulacja pojedyncza sciana
	void manipulate (common::CUBE_COLOR face_to_turn, common::CUBE_TURN_ANGLE turn_angle );

	// wykonanie sekwecji manipulacji poszczegolnymi scianami
	void execute_manipulation_sequence();
	
	//wykonanie sekwencji manipulacji w celu identyfikacji kolorow
	void identify_colors();
	
	bool communicate_with_windows_solver();
	
	// OPERACJE
	
	// obrot sciany
	void face_turn_op (common::CUBE_TURN_ANGLE turn_angle);
	// zmiana sciany (przelozenie kostki)
	void face_change_op (common::CUBE_TURN_ANGLE turn_angle);
	// dojscie
	void approach_op (int mode);
	// odejscie
	void departure_op ();


	// METODY POMOCNICZE
	
	// rozwieranie chwytakow
	void gripper_opening(double track_increment, double postument_increment, int motion_time);
	
	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);

}; // end : class nose_run_force


} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
