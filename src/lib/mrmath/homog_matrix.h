// ------------------------------------------------------------------------
// Proces:		-
// Plik:			mathtr.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Klasy K_vector, Homog_matrix, Ft_v_vector,  Jacobian_matrix
//				- definicja klas
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------


#ifndef __HOMOG_MATRIX_H
#define __HOMOG_MATRIX_H

#include <iostream>
#include <math.h>
#include <string.h>

#include "lib/impconst.h"	// frame_tab
namespace mrrocpp {
namespace lib {

// Klasa reprezentujaca macierz transformacji.
class Homog_matrix
{

private:
	// Zmienna przechowujaca parametry macierzy jednorodnej.
	frame_tab matrix_m;

public:
	// Klasa Ft_v_tr musi miec dostep do prywatnych skladnikow klasy Homog_matrix.
	friend class Ft_v_tr;
	friend class Ft_tr;
	friend class V_tr;

	// Konstruktor domniemany - tworzy macierz jednostkowa.
	Homog_matrix();
	// Stworzenie macierzy na podstawie zawartosci tablicy.
	Homog_matrix(const frame_tab &);
	// Konstruktor kopiujacy.
	Homog_matrix(const Homog_matrix &);
	// Utworzenie macierzy przesuniecia o [x, y, z], R - jednostkowa.
	Homog_matrix(double x, double y, double z);
	// Utworzenie macierzy obrotu o male katy wzgledem 3 osi.
	Homog_matrix(const K_vector & versor_x, const K_vector & versor_y, const K_vector & versor_z, const K_vector & angles);
	// Utworzenie macierzy obrotu o male katy
	Homog_matrix(const K_vector & angles);

	Homog_matrix(const Xyz_Euler_Zyz_vector & l_vector);
	Homog_matrix(const Xyz_Rpy_vector & l_vector);
	Homog_matrix(const Xyz_Angle_Axis_vector & l_vector);

	// Utworzenie macierzy jednorodnej na podstawie podanej macierzy obrotu r i wektora przesuniecia t.
	Homog_matrix(double r[3][3], double t[3]);
	// Utworzenie macierzy jednorodnej na podstawie jej 12 elementow (notacja z Craiga)
	Homog_matrix(double r11, double r12, double r13, double t1, double r21, double r22, double r23, double t2, double r31, double r32, double r33, double t3);

	Homog_matrix return_with_with_removed_translation() const;
	Homog_matrix return_with_with_removed_rotation() const;

	// Zwrocenie tablicy zawierajacej dane macierzy jednorodnej.
	void get_frame_tab(frame_tab frame) const;
	// Ustawienie tablicy zawierajacej dane macierzy jednorodnej.
	void set_from_frame_tab(const frame_tab & frame);

	// Przeksztalcenie do formy XYZ_EULER_ZYZ i zwrocenie w tablicy.
	void get_xyz_euler_zyz(Xyz_Euler_Zyz_vector & l_vector) const;

	void set_from_xyz_euler_zyz(const Xyz_Euler_Zyz_vector & l_vector);

	// Przeksztalcenie do formy XYZ_EULER_ZYZ dla robota IRP-6_MECHATRONIKA i zwrocenie w tablicy.
	void get_mech_xyz_euler_zyz(Xyz_Euler_Zyz_vector & l_vector) const;

	// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_EULER_ZYZ dla robota IRP-6_MECHATRONIKA
	void set_from_mech_xyz_euler_zyz(const Xyz_Euler_Zyz_vector & l_vector);

	// Przeksztalcenie do formy XYZ_RPY (rool pitch yaw) i zwrocenie w tablicy.
	void get_xyz_rpy(Xyz_Rpy_vector & l_vector) const;
	// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_RPY.
	void set_from_xyz_rpy(const Xyz_Rpy_vector & l_vector);

	// Przeksztalcenie do formy XYZ_ANGLE_AXIS i zwrocenie w tablicy.
	void get_xyz_angle_axis(Xyz_Angle_Axis_vector & l_vector) const;

	void set_from_xyz_angle_axis(const Xyz_Angle_Axis_vector & l_vector); // kat wliczony w os


	// Operacje na kwaternionach
	void set_from_xyz_quaternion(double eta, double eps1, double eps2, double eps3, double x, double y, double z);
	void get_xyz_quaternion(double t[7]) const;

	// Zwraca obecny wektor translacji.
	void get_translation_vector(double t[3]) const;

	// wyzerowanie wektora translacji.
	void remove_translation();

	// wstawienie jedynek na diagonalii rotacji
	void remove_rotation();

	// Ustawienie wektora translacji. Macierz rotacji pozostaje niezmieniona.
	void set_translation_vector(double t[3]);

	void set_translation_vector(double x, double y, double z);

	void set_translation_vector(const K_vector & xyz);

	void set_translation_vector(const Homog_matrix & wzor);

	// Zwrocenie macierzy rotacji.
	void get_rotation_matrix(double r[3][3]) const;
	// Ustawienie macierzy rotacji. Wektor translacji pozostaje niezmieniony.
	void set_rotation_matrix(double r[3][3]);

	void set_rotation_matrix(const Homog_matrix &wzor);

	// Ustawienie elementu macierzy.
	void set_value(int i, int j, const double value);
	// Zwrocenie elementu macierzy.
	void get_value(int i, int j, double &value) const;
	// Zwrocenie elementu macierzy.
	double get_value(int i, int j) const;

	// Operator przypisania.
	Homog_matrix & operator=(const Homog_matrix &);
	// Mnozenie macierzy.
	Homog_matrix operator*(const Homog_matrix &) const;
	// Odwracanie macierzy.
	Homog_matrix operator!() const;
	// Mnozenie macierzy i przypisanie wyniku.
	void operator*=(const Homog_matrix &);

	// operatory sluzace do przeksztalcania wektorow
	K_vector operator*(const K_vector &) const;
	K_vector operator*(const double tablica[3]) const;

	// operatory prownania macierzy jednorodnych
	int operator==(const Homog_matrix &) const;
	int operator!=(const Homog_matrix &) const;

	double* operator[](const int i);
	const double* operator[](const int i) const;

	// operator wypisania
	friend std::ostream& operator<<(std::ostream &, Homog_matrix &);

	// funkcja sprawdzajaca czy macierz jest macierza jednorodna
	int is_valid() const;

	// Kopiowanie macierzy jednorodnej do DEST z SOURCE.
	inline static void copy_frame_tab(frame_tab destination_frame, const frame_tab source_frame)
	{
		memcpy(destination_frame, source_frame, sizeof(frame_tab));
	};//: copy_frame

	// Kopiowanie macierzy jednorodnej w postaci klas pochodnych Ft_V_vector
	inline static void FT_v_vector(double destination_vector[6], double source_vector[6])
	{
		memcpy(destination_vector, source_vector, 6* sizeof (double));
	};//: copy_xyz_angle_axis


};// end class Homog_matrix


} // namespace lib
} // namespace mrrocpp

#endif
