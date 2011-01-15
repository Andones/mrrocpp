#include "base/lib/com_buf.h"

namespace mrrocpp {
namespace lib {

r_buffer::r_buffer (void) :
  reply_type(lib::ERROR),
  input_values(0)
{
  robot_model.type = TOOL_FRAME;
  arm.type = FRAME;
  error_no.error0 = OK;
  error_no.error1 = OK;

 // address_byte = 0;
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
        robot_model.tool_frame_def.tool_frame[j][i] = 1.0;
      else
        robot_model.tool_frame_def.tool_frame[j][i] = 0.0;
        */
//  robot_model.tool_frame_def.address_byte = 0;
}

} // namespace lib
} // namespace mrrocpp
