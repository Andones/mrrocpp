// ------------------------------------------------------------------------
// Proces:		MASTER PROCESS (MP)
// Plik:			generator/mp_g_two_robots_measures.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Generator odpowiedzalny za zbieranie pomiarow
//				do weryfikacji kalibracji
//				- definicja metod klasy
//
// Autor:		tkornuta
// Data:		28.02.2007
// ------------------------------------------------------------------------

#include <string.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include "mp/generator/mp_g_two_robots_measures.h"

#if defined(USE_MESSIP_SRR)
#include "lib/messip/messip_dataport.h"
#endif

namespace mrrocpp {
namespace mp {
namespace generator {

// Konstruktor.
two_robots_measures::two_robots_measures(task::task& _mp_task)
	: generator (_mp_task), UI_fd(_mp_task.UI_fd)
{
}

// Pierwszy krok generatora.
bool two_robots_measures::first_step()
{
	/*
	idle_step_counter = 2;
	// Ustawienie polecen dla robota na torze.
	irp6ot = robot_m[lib::ROBOT_IRP6_ON_TRACK];
	irp6ot->mp_command.command = lib::NEXT_POSE;
	irp6ot->mp_command.instruction.instruction_type = lib::GET;
	irp6ot->mp_command.instruction.get_type = ARM_DV;
	irp6ot->mp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
	irp6ot->mp_command.instruction.motion_type = lib::ABSOLUTE;
	irp6ot->mp_command.instruction.interpolation_type = lib::MIM;
	// Ustawienie polecen dla robota na postumencie.
	irp6p = robot_m[lib::ROBOT_IRP6_POSTUMENT];
	irp6p->mp_command.command = lib::NEXT_POSE;
	irp6p->mp_command.instruction.instruction_type = lib::GET;
	irp6p->mp_command.instruction.get_type = ARM_DV;
	irp6p->mp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
	irp6p->mp_command.instruction.motion_type = lib::ABSOLUTE;
		irp6p->mp_command.instruction.interpolation_type = lib::MIM;
	// Przepisanie polecen.

	// Wyczyszczenie listy.
	measures.clear();
	// Wyzerowanie ostatniego odczytu.
	for(int i=0; i<6; i++)
		last_measure.irp6ot[i] = 0.0;
	for(int i=0; i<6; i++)
		last_measure.irp6p[i] = 0.0;
	return true;
	*/
}


// Nastepny krok generatora.
bool two_robots_measures::next_step()
{
	// Sprawdzenie, czy nadeszlo polecenie zakonczenia zbierania pomiarow.
	if (check_and_null_trigger())
	{
		std::cout<<"Liczba zebranych pozycji: "<<measures.size()<<std::endl;
		// Zresetowanie przerwania.

		// Zapis do pliku.
		save_measures_to_file();
		return false;
	}
	// Oczekiwanie na odczyt aktualnego polozenia koncowki.
	if ( idle_step_counter )
	{
		// Przygotowanie nastepnego polecenia.

		idle_step_counter--;
		return true;
	}
	// Pobranie odczytow.

	// Przepisanie odczytow do wektora.
	two_robots_measure current_measure;
	memcpy(current_measure.irp6ot, irp6ot->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates, 6*sizeof(double));
	memcpy(current_measure.irp6p, irp6p->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates, 6*sizeof(double));
/*	std::cout<<"current_measure"<<std::endl;
	for(int i=0; i<6; i++)
		std::cout<< current_measure.irp6ot[i] <<"\t";
	for(int i=0; i<6; i++)
		std::cout<< current_measure.irp6p[i] <<"\t";
	std::cout<<std::endl;*/
	// Roznica - na kazdej wspolrzednej na razie taka sama.
	double eps = 5e-3;
	bool add_measurement = false;
	// Porownanie obecnych polozen z poprzednimi.
	for(int i=0; i<6; i++)
		if (fabs(last_measure.irp6ot[i] - current_measure.irp6ot[i]) > eps)
		{
			add_measurement = true;
			break;
		}
	// Ewenualne dodanie bierzacych pomiarow.
	if (add_measurement)
	{
		measures.push_back(current_measure);
		// Przepisanie obecnych pomiarow na poprzednie.
		memcpy(&last_measure, &current_measure, sizeof(two_robots_measure));
		// Informacja dla uzytkownika - beep.
		std::cout<<"\a"<<std::endl;
	}
	// Przygotowanie nastepnego polecenia.

	// Nastepny krok.
	return true;
}


// Zapis pomiarow do pliku.
void two_robots_measures::save_measures_to_file (void)
{
	// Przesylka z ECP do UI
	lib::ECP_message ecp_to_ui_msg;
	// Odpowiedz UI do ECP
	lib::UI_reply ui_to_ecp_rep;
	// Polecenie wprowadzenia nazwy pliku
	ecp_to_ui_msg.ecp_message = lib::SAVE_FILE;
	// Wzorzec nazwy pliku
	strcpy(ecp_to_ui_msg.string,"*.*");
	// Wyslanie polecenia pokazania okna wyboru pliku.
#if !defined(USE_MESSIP_SRR)
	ecp_to_ui_msg.hdr.type=0;
	if (MsgSend(UI_fd, &ecp_to_ui_msg, sizeof(lib::ECP_message), &ui_to_ecp_rep, sizeof(lib::UI_reply)) < 0)
#else
	if(messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0)
#endif
	{
		sr_ecp_msg.message (lib::SYSTEM_ERROR, errno, "Send to UI failed");
		throw generator::MP_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}
	// Sprawdzenie katalogu.
	if ( chdir(ui_to_ecp_rep.path) != 0 )
	{
		sr_ecp_msg.message (lib::NON_FATAL_ERROR, NON_EXISTENT_DIRECTORY);
		return;
	}
	// Otworzenie plik do zapisu.
	std::ofstream to_file(ui_to_ecp_rep.filename);
	if (!to_file.good())
	{
		sr_ecp_msg.message (lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
		return;
	}
	for(unsigned int m=0; m<measures.size(); m++)
	{
		// Pobranie danego pomiaru.
		two_robots_measure trm = (two_robots_measure)measures[m];
		for(int i=0; i<6; i++)
			to_file<< trm.irp6ot[i] <<"\t";
		for(int i=0; i<6; i++)
			to_file<< trm.irp6p[i] <<"\t";
		to_file<<"\n";
	}

	sr_ecp_msg.message("Measures were saved to file");
}


} // namespace generator
} // namespace mp
} // namespace mrrocpp
