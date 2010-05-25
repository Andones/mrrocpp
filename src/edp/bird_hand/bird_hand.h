
#include <inttypes.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

#include <string>

class Bird_hand
{
	public:
	Bird_hand();
	~Bird_hand();
	
	void connect(std::string port);
	void disconnect();
	
	void getSynchroPos(uint8_t id, int16_t &pos);
	
	void getStatus(uint8_t id, uint8_t &status, int32_t &position, int16_t &current, int16_t &torque);
	
	void getPID(uint8_t id, int16_t &p, int16_t &i, int16_t &d);
	void setPID(uint8_t id, int16_t p, int16_t i, int16_t d);
	
	void getLimit(uint8_t id, int16_t &upper, int16_t &lower);
	void setLimit(uint8_t id, int16_t upper, int16_t lower);
	
	void setCMD1(uint8_t id, int16_t t, int16_t b, int16_t Fd, int32_t rd);
	void setCMD2(uint8_t id, int16_t t, int16_t b, int16_t Fd, int32_t rd);
	void setCMD3(uint8_t id, int16_t t, int16_t b, int16_t Fd, int32_t rd);
	
	void synchronize(uint8_t id, uint16_t step);

	protected:
	private:
	
	void write_read(int fd, char* buf, unsigned int w_len, unsigned int r_len);
	
	int fd[8];
	struct termios oldtio[8];

	char buf[30];

};
