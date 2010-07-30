/*
 * bcl_message.cc
 *
 *  Created on: Jul 26, 2010
 *      Author: kszkudla
 */

#include "bcl_message.h"
#include <iostream>

namespace mrrocpp {

namespace ecp {

namespace common {

#define MP_2_ECP_STRING_SIZE 300
#define VEC_POS 22

bcl_message::bcl_message(){
}

bcl_message::~bcl_message(){
}


char* bcl_message::trajectoryToString(std::vector<double> vec){
	char *ret = new char[MP_2_ECP_STRING_SIZE];
	double* tab = reinterpret_cast<double*>(ret);

	//Write to matrix number of elements which will be written to
	tab[0] = vec.size();

	int i = 1;
	//Rewrite vector elements to matrix
	for(std::vector<double>::iterator it = vec.begin(); (it != vec.end()) && (i < MP_2_ECP_STRING_SIZE/sizeof(double)); ++it, ++i){
		tab[i] = *it;
	}

//	std::cout << "DATA TO STR " << tab[0]  << std::endl;

	return ret;
}

char* bcl_message::trajectoryToString(double& par0, double& par1, double& par2, double& par3, double& par4, double& par5, double& par6, double& par7){
	char *ret = new char[MP_2_ECP_STRING_SIZE];

	double* tab = reinterpret_cast<double*>(ret);

	tab[0] = 8;

	tab[1] = par0;
	tab[2] = par1;
	tab[3] = par2;
	tab[4] = par3;
	tab[5] = par4;
	tab[6] = par5;
	tab[7] = par6;
	tab[8] = par7;

	return ret;
}

std::vector<double> bcl_message::stringToTrajectory(char* str){

	std::vector<double> ret;

	double* tab = reinterpret_cast<double*>(str);

	ret.clear();

	std::cout << "STRING TO DATA " << ((double *)str)[0] << std::endl;

	ret.assign(tab + 1, tab + (int)tab[0]);


	return ret;
}

char* bcl_message::fradiaOrderToString(task::regions& reg, std::vector<double> vec){
	char* ret = new char[MP_2_ECP_STRING_SIZE];

	double *tab = reinterpret_cast<double*>(ret);

	switch(reg.num_found){
		case 5:
			tab[17] = reg.x_k4;
			tab[18] = reg.y_k4;
			tab[19] = reg.w_k4;
			tab[20] = reg.h_k4;
		case 4:
			tab[13] = reg.x_k3;
			tab[14] = reg.y_k3;
			tab[15] = reg.w_k3;
			tab[16] = reg.h_k3;
		case 3:
			tab[9] = reg.x_k2;
			tab[10] = reg.y_k2;
			tab[11] = reg.w_k2;
			tab[12] = reg.h_k2;
		case 2:
			tab[5] = reg.x_k1;
			tab[6] = reg.y_k1;
			tab[7] = reg.w_k1;
			tab[8] = reg.h_k1;
		case 1:
			tab[1] = reg.x_k0;
			tab[2] = reg.y_k0;
			tab[3] = reg.w_k0;
			tab[4] = reg.h_k0;
		case 0:
			tab[0] = reg.num_found;
			break;
		default:
			tab[0] = 0;
			break;
	}

	tab[VEC_POS - 1] = vec.size();

	int i = VEC_POS;
	for(std::vector<double>::iterator it = vec.begin(); (it != vec.end()) && (i < MP_2_ECP_STRING_SIZE/sizeof(double)); ++it, ++i){
		tab[i] = *it;
	}

	return ret;
}

std::vector<double> bcl_message::stringToFradiaOrder(char* str, task::regions reg){
	std::vector<double> ret;
	double *tab = reinterpret_cast<double*>(str);

	switch((int)tab[0]){
		case 5:
			reg.x_k4 = tab[17];
			reg.y_k4 = tab[18];
			reg.w_k4 = tab[19];
			reg.h_k4 = tab[20];
		case 4:
			reg.x_k3 = tab[13];
			reg.y_k3 = tab[14];
			reg.w_k3 = tab[15];
			reg.h_k3 = tab[16];
		case 3:
			reg.x_k2 = tab[9];
			reg.y_k2 = tab[10];
			reg.w_k2 = tab[11];
			reg.h_k2 = tab[12];
		case 2:
			reg.x_k1 = tab[5];
			reg.y_k1 = tab[6];
			reg.w_k1 = tab[7];
			reg.h_k1 = tab[8];
		case 1:
			reg.x_k0 = tab[1];
			reg.y_k0 = tab[2];
			reg.w_k0 = tab[3];
			reg.h_k0 = tab[4];
			reg.code_found = true;
			reg.num_found = tab[0];
			ret.assign(tab + VEC_POS, tab + (int)tab[VEC_POS - 1]);
			break;
		case 0:
		default:
			ret.clear();
			reg.code_found = false;
			break;

	}

	return ret;
}

}

}

}
