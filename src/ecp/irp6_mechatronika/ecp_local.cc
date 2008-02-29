// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_mechatronika
// 
// -------------------------------------------------------------------------

#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mis_fun.h"

#include "ecp/irp6_mechatronika/ecp_local.h"

ecp_irp6_mechatronika_robot::ecp_irp6_mechatronika_robot (configurator &_config, sr_ecp *_sr_ecp):
	ecp_robot (ROBOT_IRP6_MECHATRONIKA, _config, _sr_ecp){};
ecp_irp6_mechatronika_robot::ecp_irp6_mechatronika_robot (ecp_task& _ecp_object):
	ecp_robot (ROBOT_IRP6_MECHATRONIKA, _ecp_object){};

// --------------------------------------------------------------------------
void ecp_irp6_mechatronika_robot::create_command (void) {
  // wypelnia bufor wysylkowy do EDP na podstawie danych
  // zawartych w skladowych generatora lub warunku

  EDP_command_and_reply_buffer.instruction.instruction_type = EDP_data.instruction_type;
  EDP_command_and_reply_buffer.instruction.set_type = EDP_data.set_type;
  EDP_command_and_reply_buffer.instruction.get_type = EDP_data.get_type;
// printf("EDP_data.get_type: %d, EDP_command_and_reply_buffer.instruction.get_type: %d\n",
// EDP_data.get_type,EDP_command_and_reply_buffer.instruction.get_type);

  EDP_command_and_reply_buffer.instruction.set_rmodel_type = EDP_data.set_rmodel_type;
  EDP_command_and_reply_buffer.instruction.get_rmodel_type = EDP_data.get_rmodel_type;
  EDP_command_and_reply_buffer.instruction.set_arm_type = EDP_data.set_arm_type;
  EDP_command_and_reply_buffer.instruction.get_arm_type = EDP_data.get_arm_type;
  EDP_command_and_reply_buffer.instruction.output_values = EDP_data.output_values;

  switch (EDP_data.instruction_type) {
    case SET:
    case SET_GET:

        if (EDP_data.set_type & RMODEL_DV) {
          switch (EDP_data.set_rmodel_type) {
            case TOOL_FRAME:
            copy_frame(EDP_command_and_reply_buffer.instruction.rmodel.tool_frame_def.tool_frame_m, EDP_data.next_tool_frame_m);
              break;
            case TOOL_XYZ_ANGLE_AXIS:
              for (int j=0; j<6; j++)
                EDP_command_and_reply_buffer.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
                     = EDP_data.next_XYZ_AA_tool_coordinates[j];
              break;
            case TOOL_XYZ_EULER_ZYZ:
              for (int j=0; j<6; j++)
                EDP_command_and_reply_buffer.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
                     = EDP_data.next_XYZ_ZYZ_tool_coordinates[j];
              break;
			  ///////////////////K
			case TOOL_AS_XYZ_EULER_ZY:
              for (int j=0; j<6; j++)
                EDP_command_and_reply_buffer.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
                     = EDP_data.next_XYZ_ZYZ_tool_coordinates[j];
              break;
			  /////////////////K
            case ARM_KINEMATIC_MODEL:
                EDP_command_and_reply_buffer.instruction.rmodel.kinematic_model.kinematic_model_no
                    = EDP_data.next_kinematic_model_no;
              break;
            case SERVO_ALGORITHM:
              for (int j=0; j<IRP6_MECHATRONIKA_NUM_OF_SERVOS; j++) {
                EDP_command_and_reply_buffer.instruction.rmodel.servo_algorithm.servo_algorithm_no[j]
                   = EDP_data.next_servo_algorithm_no[j];
                EDP_command_and_reply_buffer.instruction.rmodel.servo_algorithm.servo_parameters_no[j]
                   = EDP_data.next_servo_parameters_no[j];
              }; // end: for
              break;
            default: // Blad: niewlasciwy typ modelu robota
              throw ECP_error(NON_FATAL_ERROR, INVALID_RMODEL_TYPE);
          } // end: switch (set_rmodel_type)
        }
        
         if (EDP_data.set_type & ARM_DV) {
          EDP_command_and_reply_buffer.instruction.motion_type = EDP_data.motion_type;
          EDP_command_and_reply_buffer.instruction.motion_steps = EDP_data.motion_steps;
          EDP_command_and_reply_buffer.instruction.value_in_step_no = EDP_data.value_in_step_no;
          // Wypelniamy czesc zwiazana z polozeniem ramienia
          switch (EDP_data.set_arm_type) {
            case FRAME:
            copy_frame(EDP_command_and_reply_buffer.instruction.arm.frame_def.arm_frame_m, EDP_data.next_arm_frame_m);
               break;
            case  XYZ_ANGLE_AXIS:
              for (int j=0; j<6 ; j++)
                EDP_command_and_reply_buffer.instruction.arm.coordinate_def.arm_coordinates[j]
                   = EDP_data.next_XYZ_AA_arm_coordinates[j];
             break;
            case  XYZ_EULER_ZYZ:
              for (int j=0; j<6 ; j++)
                EDP_command_and_reply_buffer.instruction.arm.coordinate_def.arm_coordinates[j]
                   = EDP_data.next_XYZ_ZYZ_arm_coordinates[j];
              break;
            case  JOINT:
              for (int j=0; j<IRP6_MECHATRONIKA_NUM_OF_SERVOS ; j++){
                EDP_command_and_reply_buffer.instruction.arm.coordinate_def.arm_coordinates[j]
                   = EDP_data.next_joint_arm_coordinates[j];
                  }
              break;
            case  MOTOR:
              for (int j=0; j<IRP6_MECHATRONIKA_NUM_OF_SERVOS ; j++)
                EDP_command_and_reply_buffer.instruction.arm.coordinate_def.arm_coordinates[j]
                   = EDP_data.next_motor_arm_coordinates[j];
              break;
            default: // Blad: niewlasciwy sposob zadawania polozenia ramienia
              throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
          } // end: (set_arm_type)

        }
      break;
    case GET:
    case SYNCHRO:
    case QUERY:
      break;
    default: // blad: nieprawidlowe polecenie
      throw ECP_error (NON_FATAL_ERROR, INVALID_ECP_COMMAND);
  } // end: switch (instruction_type)

}; // end: ecp_irp6_mechatronika_robot::create_command
// ---------------------------------------------------------------


/*---------------------------------------------------------------------*/
void ecp_irp6_mechatronika_robot::get_reply (void) {
  // pobiera z pakietu przeslanego z EDP informacje i wstawia je do
  // odpowiednich skladowych generatora lub warunku

 EDP_data.reply_type = EDP_command_and_reply_buffer.reply_package.reply_type;
  
 switch (EDP_data.reply_type) {
   case ERROR:
     EDP_data.error_no.error0 = EDP_command_and_reply_buffer.reply_package.error_no.error0;
     EDP_data.error_no.error1 = EDP_command_and_reply_buffer.reply_package.error_no.error1;
     break;
   case ACKNOWLEDGE:
     break;
   case SYNCHRO_OK:
     break;
   case ARM_INPUTS:
	get_input_reply();
   case ARM:
	get_arm_reply();
     break;
  case RMODEL_INPUTS:
	get_input_reply();
   case RMODEL:
   	get_rmodel_reply();
     break;
   case INPUTS:
	get_input_reply();
     break;
   case ARM_RMODEL_INPUTS:
   	get_input_reply();
   case ARM_RMODEL:
	get_arm_reply();
	get_rmodel_reply();
     break;
   default:  // bledna przesylka
     throw ECP_error (NON_FATAL_ERROR, INVALID_EDP_REPLY);
 }
}



void ecp_irp6_mechatronika_robot::get_input_reply (void)
{
    EDP_data.input_values = EDP_command_and_reply_buffer.reply_package.input_values;
	for (int i=0; i<8; i++) {
		EDP_data.analog_input[i]=EDP_command_and_reply_buffer.reply_package.analog_input[i];
	}
}


void ecp_irp6_mechatronika_robot::get_arm_reply (void)
{
    switch (EDP_command_and_reply_buffer.reply_package.arm_type) {
       case MOTOR:
         for (int i=0; i<IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++)
           EDP_data.current_motor_arm_coordinates[i] =
             EDP_command_and_reply_buffer.reply_package.arm.coordinate_def.arm_coordinates[i];
         break;
       case JOINT:
         for (int i=0; i<IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++)
           EDP_data.current_joint_arm_coordinates[i] =
             EDP_command_and_reply_buffer.reply_package.arm.coordinate_def.arm_coordinates[i];
               break;
       case FRAME:
       copy_frame(EDP_data.current_arm_frame_m, EDP_command_and_reply_buffer.reply_package.arm.frame_def.arm_frame_m);
        break;
       case XYZ_EULER_ZYZ:
        for (int i=0; i<6; i++)
           EDP_data.current_XYZ_ZYZ_arm_coordinates[i] =
             EDP_command_and_reply_buffer.reply_package.arm.coordinate_def.arm_coordinates[i];
         break;
         
       case XYZ_ANGLE_AXIS: 
         for (int i=0; i<6; i++)
           EDP_data.current_XYZ_AA_arm_coordinates[i] =
             EDP_command_and_reply_buffer.reply_package.arm.coordinate_def.arm_coordinates[i];
         break;

       default: // bledny typ specyfikacji pozycji
         throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
     } // end: switch (...arm_type)
}

void ecp_irp6_mechatronika_robot::get_rmodel_reply (void)
{
	switch (EDP_command_and_reply_buffer.reply_package.rmodel_type) {
		case TOOL_FRAME:
			copy_frame(EDP_data.current_tool_frame_m, EDP_command_and_reply_buffer.reply_package.rmodel.tool_frame_def.tool_frame_m);
			break;
		case TOOL_XYZ_ANGLE_AXIS:
			for (int i=0; i<6; i++)
				EDP_data.current_XYZ_AA_tool_coordinates[i] =
				    EDP_command_and_reply_buffer.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			break;
		case TOOL_XYZ_EULER_ZYZ:
			for (int i=0; i<6; i++)
				EDP_data.current_XYZ_ZYZ_tool_coordinates[i] =
				    EDP_command_and_reply_buffer.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			break;
			////////////////////K
		case TOOL_AS_XYZ_EULER_ZY:
			for (int i=0; i<6; i++)
				EDP_data.current_XYZ_ZYZ_tool_coordinates[i] =
				    EDP_command_and_reply_buffer.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			break;
			//////////////////K
		case ARM_KINEMATIC_MODEL:
			EDP_data.current_kinematic_model_no =
			    EDP_command_and_reply_buffer.reply_package.rmodel.kinematic_model.kinematic_model_no;
			break;
		case SERVO_ALGORITHM:
			for (int i=0; i<IRP6_MECHATRONIKA_NUM_OF_SERVOS; i++) {
				EDP_data.current_servo_algorithm_no[i] =
				    EDP_command_and_reply_buffer.reply_package.rmodel.servo_algorithm.servo_algorithm_no[i];
				EDP_data.current_servo_parameters_no[i] =
				    EDP_command_and_reply_buffer.reply_package.rmodel.servo_algorithm.servo_parameters_no[i];
			}
			break;
		default: // bledny typ specyfikacji modelu robota
			throw ECP_error(NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch (...rmodel_type)
}
