// ------------------------------------------------------------------------
// Proces:		MASTER PROCESS (MP)
// Plik:			mp_g_two_robots_measures.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Generator odpowiedzalny za zbieranie pomiarow
//				do weryfikacji kalibracji
//				- deklaracja klasy
//
// Autor:		tkornuta
// Data:		28.02.2007
// ------------------------------------------------------------------------

#if !defined(__MP_TRM_GEN)
#define __MP_TRM_GEN

#include <vector>
#include "mp/mp.h"

struct two_robots_measure
{
	double irp6ot[6];
	double irp6p[6];
};


// Generator odpowiedzialny za zapamietywanie na liscie polozen koncowki dwoch robotow IRp-6.
class mp_two_robots_measures_generator : public mp_generator 
{
protected:
	// Pomocnicze wskazniki na roboty.
	mp_robot *irp6ot, *irp6p;
    // Lista kinematyk.
    std::vector <two_robots_measure> measures;
    // Ostatni odczyt - do porownywania, czy pozycja jest nowa.
    two_robots_measure last_measure;
    
    const int UI_fd;
public:	
	// Konstruktor.
	mp_two_robots_measures_generator(mp_task&);
	// Pierwszy krok generatora.
	virtual bool first_step (void);
	// Nastepny krok generatora.
	virtual bool next_step (void);
	// Zapis do pliku.
	void save_measures_to_file (void);
}; //: mp_two_robots_measures_generator

#endif
