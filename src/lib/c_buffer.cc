#include "lib/com_buf.h"

namespace mrrocpp {
namespace lib {

c_buffer::c_buffer (void) :
  instruction_type(SYNCHRO),
  set_type(0),
  get_type(0),
  set_rmodel_type(TOOL_FRAME),
  get_rmodel_type(TOOL_FRAME),
  set_arm_type(FRAME),
  get_arm_type(FRAME),
  output_values(0),
  //address_byte(0),
  motion_type(ABSOLUTE),
  motion_steps(0),
  value_in_step_no(0)
{
	// konstruktor (inicjalizator) bufora polecen z ECP
/*
	for (int i=0; i<4; i++)
		for (int j=0; j<3; j++)
			if (i==j)
				arm.pf_def.arm_frame[j][i] = 1.0;
			else
				arm.pf_def.arm_frame[j][i] = 0.0;

	for (int i=0; i<4; i++)
		for (int j=0; j<3; j++)
			if (i==j)
				rmodel.tool_frame_def.tool_frame[j][i] = 1.0;
			else
				rmodel.tool_frame_def.tool_frame[j][i] = 0.0;
	// rmodel.tool_frame_def.address_byte = 0;
	 * */
}

// odczytac XXX co?
bool c_buffer::is_get_controller_state() const
{
	return get_type & CONTROLLER_STATE_DEFINITION;
}

// odczytac wejscia?
bool c_buffer::is_get_inputs() const
{
	return get_type & OUTPUTS_DEFINITION;
}

// odczytac narzedzie?
bool c_buffer::is_get_rmodel() const
{
	return get_type & RMODEL_DEFINITION;
}

// odczytac polozenie ramienia?
bool c_buffer::is_get_arm() const
{
	return get_type & ARM_DEFINITION;
}

// ustawic wyjscia?
bool c_buffer::is_set_outputs() const
{
	return set_type & OUTPUTS_DEFINITION;
}

// zmienic narzedzie?
bool c_buffer::is_set_rmodel() const
{
	return set_type & RMODEL_DEFINITION;
}

// zmienic polozenie ramienia?
bool c_buffer::is_set_arm() const
{
	return set_type & ARM_DEFINITION;
}

} // namespace lib
} // namespace mrrocpp
