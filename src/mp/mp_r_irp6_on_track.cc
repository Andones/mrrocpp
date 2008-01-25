// -------------------------------------------------------------------------
//                              mp.cc
// 
// MP Master Process - methods
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/mp_r_irp6_on_track.h"


// DLA ROBOTA IRP6_ON_TRACK

 mp_irp6_on_track_robot::mp_irp6_on_track_robot (mp_task* mp_object_l) :
	 mp_robot ( ROBOT_IRP6_ON_TRACK,  "[ecp_irp6_on_track]", mp_object_l) {}; // Konstruktor

// --------------------------------------------------------------------------
void mp_irp6_on_track_robot::create_next_pose_command (void) {
	// wypelnia bufor wysylkowy do ECP na podstawie danych
	// zawartych w skladowych generatora lub warunku
	int  j; // pomocnicze liczniki petli

  mp_command.mp_package.instruction.instruction_type = ecp_td.instruction_type;
  mp_command.mp_package.instruction.set_type = ecp_td.set_type;
  mp_command.mp_package.instruction.get_type = ecp_td.get_type;
  mp_command.mp_package.instruction.set_rmodel_type = ecp_td.set_rmodel_type;
  mp_command.mp_package.instruction.get_rmodel_type = ecp_td.get_rmodel_type;
  mp_command.mp_package.instruction.set_arm_type = ecp_td.set_arm_type;
  mp_command.mp_package.instruction.get_arm_type = ecp_td.get_arm_type;
  mp_command.mp_package.instruction.output_values = ecp_td.output_values;
  switch (ecp_td.instruction_type) {
    case SET:
    case SET_GET:
  
        if (ecp_td.set_type & RMODEL_DV) {
          switch (ecp_td.set_rmodel_type) {
            case TOOL_FRAME:
            copy_frame(mp_command.mp_package.instruction.rmodel.tool_frame_def.tool_frame_m, ecp_td.next_tool_frame_m);
                break;
            case TOOL_XYZ_ANGLE_AXIS:
              for (j=0; j<6; j++)
                mp_command.mp_package.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
                     = ecp_td.next_XYZ_AA_tool_coordinates[j];
              break;
            case TOOL_XYZ_EULER_ZYZ:
              for (j=0; j<6; j++)
                mp_command.mp_package.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
                     = ecp_td.next_XYZ_ZYZ_tool_coordinates[j];
              break;
            case ARM_KINEMATIC_MODEL:
                mp_command.mp_package.instruction.rmodel.kinematic_model.kinematic_model_no
                    = ecp_td.next_kinematic_model_no;
              break;
            case SERVO_ALGORITHM:
              for (j=0; j<IRP6_ON_TRACK_NUM_OF_SERVOS; j++) {
                mp_command.mp_package.instruction.rmodel.servo_algorithm.servo_algorithm_no[j]
                   = ecp_td.next_servo_algorithm_no[j];
                mp_command.mp_package.instruction.rmodel.servo_algorithm.servo_parameters_no[j]
                   = ecp_td.next_servo_parameters_no[j];
              }; // end: for
              break;
            default: // Blad: niewlasciwy typ modelu robota
              throw MP_error(NON_FATAL_ERROR, INVALID_RMODEL_TYPE);
          }; // end: switch (set_rmodel_type)
        }
        
         if (ecp_td.set_type & ARM_DV) { // tylko ramie
          mp_command.mp_package.instruction.motion_type = ecp_td.motion_type;
          mp_command.mp_package.instruction.motion_steps = ecp_td.motion_steps;
          mp_command.mp_package.instruction.value_in_step_no = ecp_td.value_in_step_no;
          // Wypelniamy czesc zwiazana z polozeniem ramienia
          switch (ecp_td.set_arm_type) {
            case FRAME:
            copy_frame(mp_command.mp_package.instruction.arm.frame_def.arm_frame_m, ecp_td.next_arm_frame_m);
     	        mp_command.mp_package.instruction.arm.frame_def.gripper_coordinate
	                  = ecp_td.next_gripper_coordinate; // zadany stopien rozwarcia chwytaka
                 break;
            case  XYZ_ANGLE_AXIS:
              for (j=0; j<6 ; j++)
                mp_command.mp_package.instruction.arm.coordinate_def.arm_coordinates[j]
                   = ecp_td.next_XYZ_AA_arm_coordinates[j];
        	        mp_command.mp_package.instruction.arm.coordinate_def.gripper_coordinate
	                  = ecp_td.next_gripper_coordinate; // zadany stopien rozwarcia chwytaka  
              break;
            case  XYZ_EULER_ZYZ:
              for (j=0; j<6 ; j++)
                mp_command.mp_package.instruction.arm.coordinate_def.arm_coordinates[j]
                   = ecp_td.next_XYZ_ZYZ_arm_coordinates[j];
                   mp_command.mp_package.instruction.arm.coordinate_def.gripper_coordinate
	                  = ecp_td.next_gripper_coordinate; // zadany stopien rozwarcia chwytaka  
          break;
		case  POSE_FORCE_TORQUE_AT_FRAME:
			for(int i=0;i<6;i++) {
				mp_command.mp_package.instruction.arm.pose_force_torque_at_frame_def.inertia[i]
					=ecp_td.MPtoECP_inertia[i];
				mp_command.mp_package.instruction.arm.pose_force_torque_at_frame_def.reciprocal_damping[i]
					=ecp_td.MPtoECP_reciprocal_damping[i];
				mp_command.mp_package.instruction.arm.pose_force_torque_at_frame_def.stiffness[i]
					=ecp_td.MPtoECP_stiffness[i];
					/*
				mp_command.mp_package.instruction.arm.pose_force_torque_at_frame_def.selection_vector[i]
					=ecp_td.MPselection_vector[i];
					*/

				mp_command.mp_package.instruction.arm.pose_force_torque_at_frame_def.force_xyz_torque_xyz[i]
					=ecp_td.MPtoECP_force_xyz_torque_xyz[i];
			}
			          for (int i=0; i<IRP6_ON_TRACK_NUM_OF_SERVOS ; i++)
              {
				mp_command.mp_package.instruction.arm.pose_force_torque_at_frame_def.position_velocity[i]
					=ecp_td.MPtoECP_position_velocity[i];
				mp_command.mp_package.instruction.arm.pose_force_torque_at_frame_def.stiffness_base_position[i]
					=ecp_td.MPtoECP_stiffness_base_position[i];
              }
			mp_command.mp_package.instruction.arm.pose_force_torque_at_frame_def.gripper_coordinate=ecp_td.next_gripper_coordinate;
			break;
		case  JOINT:
              for (j=0; j<IRP6_ON_TRACK_NUM_OF_SERVOS ; j++)
              {
                mp_command.mp_package.instruction.arm.coordinate_def.arm_coordinates[j]
                   = ecp_td.next_joint_arm_coordinates[j];
              }
              break;
            case  MOTOR:
              for (j=0; j<IRP6_ON_TRACK_NUM_OF_SERVOS ; j++)
                mp_command.mp_package.instruction.arm.coordinate_def.arm_coordinates[j]
                   = ecp_td.next_motor_arm_coordinates[j];
              break;
            default: // Blad: niewlasciwy sposob zadawania polozenia ramienia
              throw MP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
          }; // end: (set_arm_type)
        };
      break;
    case GET:
    case SYNCHRO:
    case QUERY:
      break;
    default: // blad: nieprawidlowe polecenie
      throw MP_error (NON_FATAL_ERROR, INVALID_ECP_COMMAND);
  }; // end: switch (instruction_type)
//       printf("%f,%f,%f,%f,%f, :%f\n",mp_command.mp_package.instruction.arm.coordinate_def.arm_coordinates[0],mp_command.mp_package.instruction.arm.coordinate_def.arm_coordinates[1],mp_command.mp_package.instruction.arm.coordinate_def.arm_coordinates[2],mp_command.mp_package.instruction.arm.coordinate_def.arm_coordinates[3],mp_command.mp_package.instruction.arm.coordinate_def.arm_coordinates[4],mp_command.mp_package.instruction.arm.coordinate_def.arm_coordinates[5] );
}; // end: mp_irp6_on_track_robot::create_next_pose_command
// ---------------------------------------------------------------


// ---------------------------------------------------------------------
void mp_irp6_on_track_robot::get_reply (void) {
  // pobiera z pakietu przeslanego z ECP informacje i wstawia je do
  // odpowiednich skladowych generatora lub warunku

 ecp_td.ecp_reply = ecp_reply.reply;
 ecp_td.reply_type = ecp_reply.ecp_reply.reply_package.reply_type;
 
//  printf("mp mp_irp6_on_track_robot get_reply:%d\n", ecp_td.reply_type);
 switch (ecp_td.reply_type) {
   case ERROR:
     ecp_td.error_no.error0 = ecp_reply.ecp_reply.reply_package.error_no.error0;
     ecp_td.error_no.error1 = ecp_reply.ecp_reply.reply_package.error_no.error1;
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
     throw MP_error (NON_FATAL_ERROR, INVALID_EDP_REPLY);
 }; // end: switch (reply_type)
}; // end: mp_irp6_on_track_robot::get_reply (robot& r)
// --------------------------------------------------------------------------


void mp_irp6_on_track_robot::get_input_reply (void)
{

	int i; // liczniki petli
	
     ecp_td.input_values = ecp_reply.ecp_reply.reply_package.input_values;
     for (i=0; i<8; i++)
	{
		ecp_td.analog_input[i]=ecp_reply.ecp_reply.reply_package.analog_input[i];
	}

}


void mp_irp6_on_track_robot::get_arm_reply (void)
{
     switch (ecp_reply.ecp_reply.reply_package.arm_type) {
       case MOTOR: {
         for (int i=0; i<IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
           ecp_td.current_motor_arm_coordinates[i] =
             ecp_reply.ecp_reply.reply_package.arm.coordinate_def.arm_coordinates[i];
         ecp_td.gripper_reg_state = 
            	ecp_reply.ecp_reply.reply_package.arm.coordinate_def.gripper_reg_state;
         break;
       };
       case JOINT: {
         for (int i=0; i<IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
           ecp_td.current_joint_arm_coordinates[i] =
             ecp_reply.ecp_reply.reply_package.arm.coordinate_def.arm_coordinates[i];
        ecp_td.gripper_reg_state = 
            	ecp_reply.ecp_reply.reply_package.arm.coordinate_def.gripper_reg_state;
         break;
       };
       case FRAME: {
       copy_frame(ecp_td.current_arm_frame_m, ecp_reply.ecp_reply.reply_package.arm.frame_def.arm_frame_m);
       ecp_td.gripper_reg_state = 
            	ecp_reply.ecp_reply.reply_package.arm.frame_def.gripper_reg_state;
	ecp_td.current_gripper_coordinate =
             ecp_reply.ecp_reply.reply_package.arm.frame_def.gripper_coordinate;
         break;
       };
       case XYZ_EULER_ZYZ: {
         for (int i=0; i<6; i++)
           ecp_td.current_XYZ_ZYZ_arm_coordinates[i] =
             ecp_reply.ecp_reply.reply_package.arm.coordinate_def.arm_coordinates[i];
          ecp_td.gripper_reg_state = 
            	ecp_reply.ecp_reply.reply_package.arm.coordinate_def.gripper_reg_state;
           ecp_td.current_gripper_coordinate =
             ecp_reply.ecp_reply.reply_package.arm.coordinate_def.gripper_coordinate;
         break;
       };
	case POSE_FORCE_TORQUE_AT_FRAME:
			copy_frame(ecp_td.MPcurrent_beggining_arm_frame_m,
							ecp_reply.ecp_reply.reply_package.arm.pose_force_torque_at_frame_def.beggining_arm_frame_m);
			copy_frame(ecp_td.MPcurrent_predicted_arm_frame_m,
							ecp_reply.ecp_reply.reply_package.arm.pose_force_torque_at_frame_def.predicted_arm_frame_m);
			copy_frame(ecp_td.MPcurrent_present_arm_frame_m,
							ecp_reply.ecp_reply.reply_package.arm.pose_force_torque_at_frame_def.present_arm_frame_m);
			for(int i = 0;i<6;i++) {
				ecp_td.ECPtoMP_force_xyz_torque_xyz[i] = 
				ecp_reply.ecp_reply.reply_package.arm.pose_force_torque_at_frame_def.force_xyz_torque_xyz[i];
			}
			ecp_td.gripper_reg_state = ecp_reply.ecp_reply.reply_package.arm.pose_force_torque_at_frame_def.gripper_reg_state;
			ecp_td.current_gripper_coordinate = ecp_reply.ecp_reply.reply_package.arm.pose_force_torque_at_frame_def.gripper_coordinate;
		break;
       case XYZ_ANGLE_AXIS: {
         for (int i=0; i<6; i++)
           ecp_td.current_XYZ_AA_arm_coordinates[i] =
             ecp_reply.ecp_reply.reply_package.arm.coordinate_def.arm_coordinates[i];
           ecp_td.gripper_reg_state = 
            	ecp_reply.ecp_reply.reply_package.arm.coordinate_def.gripper_reg_state;
           ecp_td.current_gripper_coordinate =
             ecp_reply.ecp_reply.reply_package.arm.coordinate_def.gripper_coordinate;
         break;
       };
       default: // bledny typ specyfikacji pozycji
         throw MP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
     }; // end: switch (...arm_type)
}


void mp_irp6_on_track_robot::get_rmodel_reply (void)
{

	int i,j; // liczniki petli
	
   switch (ecp_reply.ecp_reply.reply_package.rmodel_type) {
       case TOOL_FRAME:
       copy_frame(ecp_td.current_tool_frame_m, ecp_reply.ecp_reply.reply_package.rmodel.tool_frame_def.tool_frame_m);
         break;
       case TOOL_XYZ_ANGLE_AXIS:
         for (i=0; i<6; i++)
           ecp_td.current_XYZ_AA_tool_coordinates[i] =
             ecp_reply.ecp_reply.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
         break;
       case TOOL_XYZ_EULER_ZYZ:
         for (i=0; i<6; i++)
           ecp_td.current_XYZ_ZYZ_tool_coordinates[i] =
             ecp_reply.ecp_reply.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
         break;
       case ARM_KINEMATIC_MODEL:
         ecp_td.current_kinematic_model_no =
           ecp_reply.ecp_reply.reply_package.rmodel.kinematic_model.kinematic_model_no;
         break;
       case SERVO_ALGORITHM:
         for(j=0; j<IRP6_ON_TRACK_NUM_OF_SERVOS; j++) {
           ecp_td.current_servo_algorithm_no[j] =
             ecp_reply.ecp_reply.reply_package.rmodel.servo_algorithm.servo_algorithm_no[j];
           ecp_td.current_servo_parameters_no[j] =
             ecp_reply.ecp_reply.reply_package.rmodel.servo_algorithm.servo_parameters_no[j];
         };
         break;
       default: // bledny typ specyfikacji modelu robota
         throw MP_error(NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
     }; // end: switch (...rmodel_type)
}
