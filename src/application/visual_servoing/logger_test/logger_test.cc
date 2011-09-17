#include <iostream>
#include <sstream>
#include <unistd.h>

#include "base/lib/logger/logger.h"

using namespace logger;
using namespace std;


int main(int argc, char *argv[])
{
	{
	cout<<"1\n";

	logger_client log(3, "localhost", 7000);

	for(int i=0; i<1000; ++i){
		//usleep(1e3);
		log_message lm;
		stringstream ss;
		ss<<i;

		sprintf(lm.text, "Message %d", i);
		log.log(lm);
	}
	cout<<"2\n";
	}
	cout<<"3\n";

	return 0;
}

