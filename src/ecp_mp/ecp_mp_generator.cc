#include "ecp_mp/ecp_mp_generator.h"

ecp_mp_generator::ecp_mp_generator(sr_ecp& _sr_ecp_msg) :
        sr_ecp_msg(_sr_ecp_msg),
        trigger(false),
        node_counter(0)
{}


bool ecp_mp_generator::check_and_null_trigger()
{
    bool returned_value = false;
    if (trigger)
    {
        trigger = false;
        returned_value = true;
    }

    return returned_value;
}


ecp_mp_generator::~ecp_mp_generator()
{}
