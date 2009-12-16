 // -------------------------------------------------------------------------
//                            vsp_s.cc 		dla QNX6.2
//
//            Virtual Sensor Process (lib::VSP) - methods
// Metody klasy VSP
//
// Ostatnia modyfikacja: 25.06.03
// Autor: tkornuta
// odrem - prywrocic pry podlaczeniu klasy kamera
// -------------------------------------------------------------------------

#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vis_pbeclsac/vsp_vis_pbeclsac.h"
#include "vsp/vis/cmvision.h"
#include "vsp/vis/cube.h"
#include "vsp/vis/global.h"
#include "vsp/vis/calib.h"
#include "vsp/vis/macierze_nr.h"

// Konfigurator
#include "lib/configurator.h"

int alloc_m=0, alloc_v=0; // globalnie widoczne liczby zaalokowanych macierzy i wektorow
namespace mrrocpp {
namespace vsp {
namespace sensor {


#define XMAX 768
#define YMAX 576

int ImageBPL = 1024;
int state = 0;
int fd;
unsigned short buffer[600000];



int size_read;
clock_t start_time, end_time;

// #pragma off(check_stack);
int interatt=0;
int x=0;
int z=0;
int id;
int md;
struct timespec start[9], stop[9], res;
//short tmp[9];

/****7****/

float timex;
float timex1;


struct timespec crr_time, s_time, e_time;

int debug=0;



int ret=0;
CMVision vision;
RubiksCube k1,k2;

// #pragma on(check_stack);

// extern pid_t UI_pid;           // identyfikator procesu UI


// extern lib::configurator* config;

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
sensor* return_created_sensor (lib::configurator &_config)
{
	return new vis_pbeclsac(_config);
}// : return_created_sensor




// Rejstracja procesu VSP
vis_pbeclsac::vis_pbeclsac(lib::configurator &_config) : sensor(_config){
	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.camera);

	is_sensor_configured=false;	// czujnik niezainicjowany
	is_reading_ready=false;				// nie ma zadnego gotowego odczytu

	std::string colors_file(mrrocpp_network_path);
	colors_file += "data/color_eih.txt";

	std::string pattern_file(mrrocpp_network_path);
	pattern_file += "data/pattern.txt";

	if (vision.loadColors(colors_file.c_str())){
		vision.initialize(XMAX,YMAX);
		vision.countLUT();
		vision.initEstim(pattern_file.c_str());
		vision.initGrid();
	}
	else
	{
		printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
	}

	fd = open("/dev/bttvx",O_RDWR); // bezposrednio odczyt ze sterownika zamiast konstruktora

	z=0;
	x=0;

	sr_msg->message ("VSP VIS PB-ECL-SAC started");
}

vis_pbeclsac::~vis_pbeclsac(void){
	close (fd);


	printf("Destruktor VSP\n");
}

/**************************** inicjacja czujnika ****************************/
void vis_pbeclsac::configure_sensor (void){

	is_sensor_configured=true;

     sr_msg->message ("Sensor initiated"); // 7
	}

/*************************** inicjacja odczytu ******************************/
void vis_pbeclsac::initiate_reading (void){
// printf("7 - initiate reading\n");

	if(!is_sensor_configured)
	     throw sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);

clock_gettime( CLOCK_REALTIME , &s_time);



	size_read = read( fd, buffer, sizeof( buffer ) ); // bezposredni odczyt zamiast przez klase

	lseek(fd,0,SEEK_SET);

//recog
  vision.findBlobs(buffer);

vision.filterBlobsReset();
vision.filterBlobs(BLOB_SIZE_BIGGER,200.0);
vision.filterBlobs(BLOB_SIZE_SMALLER,10000.0);
vision.findVerticesAll();
vision.filterBlobs(VERTICES_BIGGER,3.0);
vision.filterBlobs(VERTICES_SMALLER,7.0);
vision.filterBlobs(BLOB_CIRCULARITY_BIGGER,0.5);
vision.filterBlobs(BLOB_CIRCULARITY_SMALLER,6.0);


vision.estimPose3();


 k1.clear();

if(k1.build(&vision))	vision.setRoi(k1.roi,40);
else
vision.setRoi(k1.roi,1000);


// koniec przepisywania
	is_reading_ready=true;							// odczyt jakikolwiek


	}

/***************************** odczyt z czujnika *****************************/
void vis_pbeclsac::get_reading (void){
// printf("7 - get reading\n");
	if(!is_sensor_configured)
	     throw sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// jezeli chcemy jakikolwiek odczyt	-> is_reading_ready
	if(!is_reading_ready)
	     throw sensor_error (lib::FATAL_ERROR, READING_NOT_READY);

	from_vsp.vsp_report= lib::VSP_REPLY_OK;
	// tutaj: czujnik skalibrowany, odczyt dokonany, zapisany w "image", przepisanie wszystkich pol
	// przepisanie do bufora komunikacyjnego
double aux=0;

	// fill up frame
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
			vision.E_Tx_G.get_value(j,i,aux);
			from_vsp.comm_image.sensor_union.camera.frame[4*i+j]=aux;
		}
	for(int i=0; i<3; i++)
	{
			vision.E_Tx_G.get_value(3,i,aux);
			from_vsp.comm_image.sensor_union.camera.frame[4*i+3]=aux;
	}
	for(int j=0; j<3; j++)
			from_vsp.comm_image.sensor_union.camera.frame[12+j]=0;
	if (vision.whole_face)
			from_vsp.comm_image.sensor_union.camera.frame[15]=1;
	else
			from_vsp.comm_image.sensor_union.camera.frame[15]=0;
	// for(int i=0; i<16; i++)
	// 	from_vsp.comm_image.sensor_union.camera.frame[i] = 0.5;
     // sr_msg->message ("VSP Get reading ok");
     is_reading_ready=false; // 7
	}

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp
