/*
 * epos_access_socketcan.cc
 *
 *  Created on: May 14, 2011
 *      Author: ptroja
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "epos_access_socketcan.h"

namespace mrrocpp {
namespace edp {
namespace epos {

epos_access_socketcan::epos_access_socketcan(const std::string & _iface) :
	iface(iface)
{
	if(iface.length() >= IFNAMSIZ) {
		throw epos_error() << reason("name of CAN device too long");
	}
}

epos_access_socketcan::~epos_access_socketcan()
{
	if (device_opened) close();
}

void epos_access_socketcan::open()
{
	/* Create the socket */
	int sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (sock == -1) {
		perror("socket()");
		throw epos_error() << reason("failed to create a CAN socket");
	}

	/* Locate the interface you wish to use */
	struct ifreq ifr;
	// FIXME: use strncpy
	strcpy(ifr.ifr_name, iface.c_str());
	::ioctl(sock, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled
	 * with that device's index */

	/* Select that CAN interface, and bind the socket to it. */
	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if(::bind(sock, (struct sockaddr*) &addr, sizeof(addr)) == -1) {
		perror("bind()");
		throw epos_error() << reason("failed to bind to a CAN interface");
	}

	device_opened = true;
}

void epos_access_socketcan::close()
{
	if(::close(sock) == -1) {
		throw epos_error() << reason("failed to close CAN socket");;
	}

	device_opened = false;
}

canid_t epos_access_socketcan::readFromWire(struct can_frame & frame)
{
	int nbytes;

    /* read frame */
    if ((nbytes = ::read(sock, &frame, sizeof(frame))) != sizeof(frame)) {
        perror("read()");
        BOOST_THROW_EXCEPTION(epos_error() << reason("read from CAN socket failed"));
    }

    return (frame.can_id);
}

void epos_access_socketcan::writeToWire(const struct can_frame & frame)
{
	int nbytes;

    /* send frame */
    if ((nbytes = ::write(sock, &frame, sizeof(frame))) != sizeof(frame)) {
        perror("write()");
        BOOST_THROW_EXCEPTION(epos_error() << reason("write to CAN socket failed"));
    }
}

void epos_access_socketcan::handleCanOpenMgmt(const struct can_frame & frame)
{

}

unsigned int epos_access_socketcan::ReadObject(WORD *ans, unsigned int ans_len, uint8_t nodeId, WORD index, BYTE subindex)
{
	struct can_frame frame;

	frame.can_id = 0x600 + nodeId;

	frame.data[0] = 0x40;	// Initiate Domain Upload, client => server
	frame.data[1] = (index & 0xFF); // index high BYTE
	frame.data[2] = (index >> 8);   // index low BYTE
	frame.data[3] = subindex;
	frame.data[4] = 0;	// don't care, should be zero
	frame.data[5] = 0;
	frame.data[6] = 0;
	frame.data[7] = 0;

	return 0;
}

#if 0
/*! NOT USED IN libEPOS so far -> untested!
 */
static int InitiateSegmentedRead(WORD index, BYTE subindex ) {

	WORD frame[4], **ptr;

	frame[0] = 0x1201; // fixed, opCode==0x12, (len-1) == 1
	frame[1] = index;
	frame[2] = 0x0000 | subindex; /* high BYTE: 0x00 (Node-ID == 0)
	 low BYTE: subindex */
	frame[3] = 0x000; // ZERO word, will be filled with checksum

	sendCommand(frame);

	// read response
	return( readAnswer(ptr) ); // answer contains only DWORD ErrorCode
	// here...
}

/*! NOT USED IN libEPOS so far -> untested! */
static int SegmentRead(WORD **ptr) {

	WORD frame[3];
	int n;

	frame[0] = 0x1400; // fixed, opCode==0x14, (len-1) == 0
	frame[1] = 0x0000; // WHAT IS THE 'TOGGLE' BIT????
	frame[2] = 0x0000; // ZERO word, will be filled with checksum

	sendCommand(frame);

	readAnswer(ptr);

	return(0);
}
#endif

void epos_access_socketcan::WriteObject(uint8_t nodeId, WORD index, BYTE subindex, uint32_t data)
{
	struct can_frame frame;

	frame.can_id = 0x600 + nodeId;

	frame.data[0] = 0x40;	// Initiate Domain Upload, client => server
	frame.data[1] = (index & 0xFF); // index high BYTE
	frame.data[2] = (index >> 8);   // index low BYTE
	frame.data[3] = subindex;
	frame.data[4] = 0;	// don't care, should be zero
	frame.data[5] = 0;
	frame.data[6] = 0;
	frame.data[7] = 0;
}

void epos_access_socketcan::InitiateSementedWrite(uint8_t nodeId, WORD index, BYTE subindex, DWORD ObjectLength)
{
	struct can_frame frame;

	frame.can_id = 0x600 + nodeId;

	frame.data[0] = 0x40;	// Initiate Domain Upload, client => server
	frame.data[1] = (index & 0xFF); // index high BYTE
	frame.data[2] = (index >> 8);   // index low BYTE
	frame.data[3] = subindex;
	frame.data[4] = 0;	// don't care, should be zero
	frame.data[5] = 0;
	frame.data[6] = 0;
	frame.data[7] = 0;
}

void epos_access_socketcan::SegmentedWrite(uint8_t nodeId, BYTE * ptr, std::size_t len)
{
	if (len > 63) {
		BOOST_THROW_EXCEPTION(epos_error() << reason("Segmented write of > 63 bytes not allowed"));
	}

	struct can_frame frame;

	frame.can_id = 0x600 + nodeId;

	frame.data[0] = 0x40;	// Initiate Domain Upload, client => server
}

void epos_access_socketcan::SendNMTService(uint8_t nodeId, NMT_COMMAND_t CmdSpecifier)
{
	struct can_frame frame;

	frame.can_id = 0x600 + nodeId;

	frame.data[0] = 0x40;	// Initiate Domain Upload, client => server

	writeToWire(frame);
}

void epos_access_socketcan::SendCANFrame(WORD Identifier, WORD Length, const BYTE Data[8])
{
	struct can_frame frame;

	frame.can_id = Identifier;

	memcpy(frame.data, Data, Length);

	writeToWire(frame);
}


} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */
