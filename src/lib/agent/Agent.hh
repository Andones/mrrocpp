#ifndef __AGENT_HH
#define __AGENT_HH

#include <string>

#include <boost/unordered_map.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#if defined(USE_MESSIP_SRR)
#include "lib/messip/messip.h"
#else /* USE_MESSIP_SRR */
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#endif /* USE_MESSIP_SRR */

#include "AgentBase.hh"

// forward declarations
class DataBufferBase;
template <class T> class DataBuffer;
class OrBufferContainer;
class AndBufferContainer;

/**
 * Agent base class
 */
class Agent : public AgentBase {
private:
	//! check if given data availability condition is satisfied
	bool checkCondition(const OrBufferContainer &condition);

	/**
	 * Receive data buffer message
	 * @param block until a message arrive
	 * @return true if the a message has arrived
	 */
	bool ReceiveMessage(bool block);

#if defined(USE_MESSIP_SRR)
	//! server channel id
	messip_channel_t * channel;
#else
	//! server channel id
	name_attach_t * channel;
#endif

	//! thread id of the of the non-blocking receive implementation
	boost::thread * tid;

	//! condition variable for synchronization wake-up after receiving data
	boost::condition_variable cond;

	//! mutex for protection data between receiver and readers
	mutable boost::mutex mtx;

	//! Data receiver thread loop
	void ReceiveDataLoop(void);

protected:
	//! Datatype of buffers container
	typedef boost::unordered_map<std::string, DataBufferBase * > buffers_t;

	//! Datatype of buffers container value
	typedef buffers_t::value_type buffer_item_t;

	//! Buffer container
	buffers_t buffers;

	//! Add a buffer to the agent
	void registerBuffer(DataBufferBase & buf);

	//! List buffers of the agent
	void listBuffers() const;

protected:
	/**
	 * Wait for given data availability condition to be satisfied
	 * @param orCondition condition to wait for
	 */
	void Wait(OrBufferContainer & orCondition);

	void Wait(AndBufferContainer & orCondition);

	void Wait(DataBufferBase & dataBufferCondition);

public:
	//! Constructor
	Agent(const std::string & _name);

	//! Destructor
	virtual ~Agent();

	//! Single step of agent's transition function
	virtual bool step() = 0;

	//! Main loop of the agent
	void operator ()();
};

#endif /* __AGENT_HH */
