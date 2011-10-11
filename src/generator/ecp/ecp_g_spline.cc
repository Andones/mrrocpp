/**
 * @file
 * @brief Contains definitions of the methods of spline class.
 * @author rtulwin
 * @ingroup generators
 */

#include <fstream>
#include "ecp_g_spline.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

using namespace std;

spline::spline(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
        multiple_position<ecp_mp::common::trajectory_pose::spline_trajectory_pose,
        ecp::common::generator::trajectory_interpolator::spline_interpolator,
        ecp::common::generator::velocity_profile_calculator::spline_profile> (_ecp_task) {

    this->pose_spec = pose_spec;
    this->axes_num = axes_num;
    this->vpc = velocity_profile_calculator::spline_profile();
    this->inter = trajectory_interpolator::spline_interpolator();
    this->type = cubic;

    create_velocity_vectors(axes_num);
}

spline::~spline() {

}

void spline::print_pose_vector() {

}

bool spline::calculate()
{
    sr_ecp_msg.message("Calculating...");

    int i,j;//loop counter

    pose_vector_iterator = pose_vector.begin();

    for (i = 0; i < pose_vector.size(); i++) {//calculate distances, directions, times and velocities for each pose and axis

            if(motion_type == lib::ABSOLUTE) {//absolute type of motion
                    if (!vpc.calculate_absolute_distance_direction_pose(pose_vector_iterator)) {
                            return false;
                    }
            } else if(motion_type == lib::RELATIVE) {//relative type of motion
                    if (!vpc.calculate_relative_distance_direction_pose(pose_vector_iterator)) {
                            return false;
                    }
            } else {
                    sr_ecp_msg.message("Wrong motion type");
                    throw ECP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);//TODO change the second argument
            }

            if(!vpc.calculate_time_pose(pose_vector_iterator) ||//calculate times for each of the axes
               !vpc.calculate_pose_time(pose_vector_iterator, mc)) {//calculate the longest time from each of the axes and set it as the pose time (also extend the time to be the multiplcity of a single macrostep time)
                    return false;
            }

            for (j = 0; j < axes_num; j++) {
                if (pose_vector_iterator->type == 1) {
                    if (!vpc.calculate_linear_coeffs(pose_vector_iterator,j)) {
                        return false;
                    }
                } else if (pose_vector_iterator->type == 2) {
                    if (!vpc.calculate_cubic_coeffs(pose_vector_iterator,j)) {
                        return false;
                    }
                } else if (pose_vector_iterator->type == 3) {
                    if (!vpc.calculate_quintic_coeffs(pose_vector_iterator,j)) {
                        return false;
                    }
                }
            }

            //calculate the number of the macrosteps for the pose
            pose_vector_iterator->interpolation_node_no = ceil(pose_vector_iterator->t / mc);

            if (debug) {
                    printf("interpolation node no: %d\n", pose_vector_iterator->interpolation_node_no);
            }

            pose_vector_iterator++;
    }

    return true;
}

void spline::create_velocity_vectors(int axes_num) {
    joint_velocity = vector<double>(axes_num, 0.15);
    joint_max_velocity = vector<double>(axes_num, 1.5);
    joint_acceleration = vector<double>(axes_num, 0.02);
    joint_max_acceleration = vector<double>(axes_num, 7.0);
    motor_velocity = vector<double>(axes_num, 0.15);
    motor_max_velocity = vector<double>(axes_num, 200.0);
    motor_acceleration = vector<double>(axes_num, 0.02);
    motor_max_acceleration = vector<double>(axes_num, 150.0);
    euler_zyz_velocity= vector<double>(axes_num, 0.15);//TODO check if this is a reasonable value
    euler_zyz_max_velocity = vector<double>(axes_num, 5.0);
    euler_zyz_acceleration = vector<double>(axes_num, 0.02);//TODO check if this is a reasonable value
    euler_zyz_max_acceleration = vector<double>(axes_num, 5.0);
    angle_axis_velocity = vector<double>(axes_num, 0.15);//TODO check if this is a reasonable value
    angle_axis_max_velocity = vector<double>(axes_num, 5.0);
    angle_axis_acceleration = vector<double>(axes_num, 0.02);//TODO check if this is a reasonable value
    angle_axis_max_acceleration = vector<double>(axes_num, 5.0);
}

//--------------- METHODS USED TO LOAD POSES ----------------

bool spline::load_absolute_joint_trajectory_pose(const vector<double> & coordinates) {

        ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

        return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_JOINT, joint_velocity, joint_acceleration, joint_max_velocity, joint_max_acceleration);
}

bool spline::load_relative_joint_trajectory_pose(const vector<double> & coordinates) {

        ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

        return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_JOINT, joint_velocity, joint_acceleration, joint_max_velocity, joint_max_acceleration);
}

bool spline::load_absolute_motor_trajectory_pose(const vector<double> & coordinates) {

        ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

        return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_MOTOR, motor_velocity, motor_acceleration, motor_max_velocity, motor_max_acceleration);
}

bool spline::load_relative_motor_trajectory_pose(const vector<double> & coordinates) {

        ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

        return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_MOTOR, motor_velocity, motor_acceleration, motor_max_velocity, motor_max_acceleration);
}

bool spline::load_absolute_euler_zyz_trajectory_pose(const vector<double> & coordinates) {

        ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

        return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_XYZ_EULER_ZYZ, euler_zyz_velocity, euler_zyz_acceleration, euler_zyz_max_velocity, euler_zyz_max_acceleration);
}

bool spline::load_relative_euler_zyz_trajectory_pose(const vector<double> & coordinates) {

        ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

        return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_XYZ_EULER_ZYZ, euler_zyz_velocity, euler_zyz_acceleration, euler_zyz_max_velocity, euler_zyz_max_acceleration);
}

bool spline::load_absolute_angle_axis_trajectory_pose(const vector<double> & coordinates) {

        ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

        return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_XYZ_ANGLE_AXIS, angle_axis_velocity, angle_axis_acceleration, angle_axis_max_velocity, angle_axis_max_acceleration);
}

bool spline::load_relative_angle_axis_trajectory_pose(const vector<double> & coordinates) {

        ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

        return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_XYZ_ANGLE_AXIS, angle_axis_velocity, angle_axis_acceleration, angle_axis_max_velocity, angle_axis_max_acceleration);
}

bool spline::load_absolute_pose(ecp_mp::common::trajectory_pose::spline_trajectory_pose & trajectory_pose) {

    if (trajectory_pose.arm_type == lib::ECP_JOINT) {
            load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a,  joint_max_velocity, joint_max_acceleration);
    } else if (trajectory_pose.arm_type == lib::ECP_MOTOR) {
            load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a,  motor_max_velocity, motor_max_acceleration);
    } else if (trajectory_pose.arm_type == lib::ECP_XYZ_ANGLE_AXIS) {
            load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a,  angle_axis_max_velocity, angle_axis_max_acceleration);
    } else if (trajectory_pose.arm_type == lib::ECP_XYZ_EULER_ZYZ) {
            load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a,  euler_zyz_max_velocity, euler_zyz_max_acceleration);
    } else {
            throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    }
    return true;
}

bool spline::load_relative_pose(ecp_mp::common::trajectory_pose::spline_trajectory_pose & trajectory_pose) {

    if (trajectory_pose.arm_type == lib::ECP_JOINT) {
            load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a,  joint_max_velocity, joint_max_acceleration);
    } else if (trajectory_pose.arm_type == lib::ECP_MOTOR) {
            load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a,  motor_max_velocity, motor_max_acceleration);
    } else if (trajectory_pose.arm_type == lib::ECP_XYZ_ANGLE_AXIS) {
            load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a,  angle_axis_max_velocity, angle_axis_max_acceleration);
    } else if (trajectory_pose.arm_type == lib::ECP_XYZ_EULER_ZYZ) {
            load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a,  euler_zyz_max_velocity, euler_zyz_max_acceleration);
    } else {
            throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
    }
    return true;
}

bool spline::load_trajectory_pose(const vector<double> & coordinates, lib::MOTION_TYPE motion_type, lib::ECP_POSE_SPECIFICATION pose_spec, const vector<double> & v, const vector<double> & a, const vector<double> & v_max, const vector<double> & a_max)
{
    if (!pose_vector.empty() && this->pose_spec != pose_spec) { //check if previous positions were provided in the same representation

            sr_ecp_msg.message("Representation different than the previous one");
            return false;
    }

    if (!pose_vector.empty() && this->motion_type != motion_type) {

            sr_ecp_msg.message("Wrong motion type");
            return false;
    }

    this->motion_type = motion_type;
    this->pose_spec = pose_spec;

    ecp_mp::common::trajectory_pose::spline_trajectory_pose pose; //new trajectory pose
    pose = ecp_mp::common::trajectory_pose::spline_trajectory_pose(pose_spec, coordinates, v, a); //create new trajectory pose
    pose.v_max = v_max; //set the v_max vector
    pose.a_max = a_max; //set the a_max vector

    for (int j = 0; j < axes_num; j++) { //calculate v_r velocities
            pose.v_r[j] = pose.v[j] * pose.v_max[j];
            pose.a_r[j] = pose.a[j] * pose.a_max[j];
    }

    if (pose_vector.empty()) {
            pose.pos_num = 1;
    } else {
            pose.pos_num = pose_vector.back().pos_num + 1;
    }

    if (motion_type == lib::ABSOLUTE) {
            if (!pose_vector.empty()) {//set the start position of the added pose as the desired position of the previous pose
                    pose.start_position = pose_vector.back().coordinates;
            }
    }

    pose_vector.push_back(pose); //put new trajectory pose into a pose vector

    sr_ecp_msg.message("Pose loaded");

    return true;
}

bool spline::load_trajectory_from_file(const char* file_name) {

        sr_ecp_msg.message(file_name);

        char coordinate_type_desc[80]; //description of pose specification read from the file
        char motion_type_desc[80]; //description of motion type read from the file
        lib::ECP_POSE_SPECIFICATION ps; //pose specification read from the file
        lib::MOTION_TYPE mt; //type of the commanded motion (relative or absolute)
        int number_of_poses = 0; //number of poses to be read
        int i, j; //loop counters


        std::ifstream from_file(file_name); // open the file
        if (!from_file.good()) {
                //perror(file_name);
                throw ECP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
                return false;
        }

        if (!(from_file >> coordinate_type_desc)) {
                throw ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
                return false;
        }

        //removing spaces and tabs
        i = 0;
        j = 0;
        while (coordinate_type_desc[i] == ' ' || coordinate_type_desc[i] == '\t')
                i++;
        while (coordinate_type_desc[i] != ' ' && coordinate_type_desc[i] != '\t' && coordinate_type_desc[i] != '\n' && coordinate_type_desc[i]
                        != '\r' && coordinate_type_desc[j] != '\0') {
                coordinate_type_desc[j] = toupper(coordinate_type_desc[i]);
                i++;
                j++;
        }
        coordinate_type_desc[j] = '\0';

        if (!strcmp(coordinate_type_desc, "MOTOR")) {
                ps = lib::ECP_MOTOR;
        } else if (!strcmp(coordinate_type_desc, "JOINT")) {
                ps = lib::ECP_JOINT;
        } else if (!strcmp(coordinate_type_desc, "XYZ_EULER_ZYZ")) {
                ps = lib::ECP_XYZ_EULER_ZYZ;
        } else if (!strcmp(coordinate_type_desc, "XYZ_ANGLE_AXIS")) {
                ps = lib::ECP_XYZ_ANGLE_AXIS;
        } else {
                throw ECP_error(lib::NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
                return false;
        }



        if (!(from_file >> number_of_poses)) {
                throw ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
                return false;
        }

        if (!(from_file >> motion_type_desc)) {
                throw ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
                return false;
        }

        if (!strcmp(motion_type_desc, "ABSOLUTE")) {
                mt = lib::ABSOLUTE;
        } else if (!strcmp(motion_type_desc, "RELATIVE")) {
                mt = lib::RELATIVE;
        } else {
                throw ECP_error(lib::NON_FATAL_ERROR, NON_TRAJECTORY_FILE);
                return false;
        }

        double tab[10];
        int pos = from_file.tellg();
        char line[80];
                int dlugosc;
                do
                {
                from_file.getline(line, 80);
                dlugosc=strlen(line);
                }
                while (dlugosc<5);
int num = lib::setValuesInArray(tab,line);
this->set_axes_num(num);
from_file.seekg(pos);

std::vector <double> v(axes_num); //vector of read velocities
std::vector <double> a(axes_num); //vector of read accelerations
std::vector <double> coordinates(axes_num); //vector of read coordinates

        for (i = 0; i < number_of_poses; i++) {
                for (j = 0; j < axes_num; j++) {
                        if (!(from_file >> v[j])) { //protection before the non-numerical data
                                throw ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
                                return false;
                        }
                }
                from_file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                for (j = 0; j < axes_num; j++) {
                        if (!(from_file >> a[j])) { //protection before the non-numerical data
                                throw ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
                                return false;
                        }
                }
                from_file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                for (j = 0; j < axes_num; j++) {
                        if (!(from_file >> coordinates[j])) { //protection before the non-numerical data
                                throw ECP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
                                return false;
                        }
                }
                from_file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                if (ps == lib::ECP_MOTOR) {
                        load_trajectory_pose(coordinates, mt, ps, v, a, motor_max_velocity, motor_max_acceleration);
                } else if (ps == lib::ECP_JOINT) {
                        load_trajectory_pose(coordinates, mt, ps, v, a, joint_max_velocity, joint_max_acceleration);
                } else if (ps == lib::ECP_XYZ_EULER_ZYZ) {
                        load_trajectory_pose(coordinates, mt, ps, v, a, euler_zyz_max_velocity, euler_zyz_max_acceleration);
                } else if (ps == lib::ECP_XYZ_ANGLE_AXIS) {
                        load_trajectory_pose(coordinates, mt, ps, v, a, angle_axis_max_velocity, angle_axis_max_acceleration);
                }
        }

        return true;
}

//--------------- METHODS USED TO LOAD POSES END ----------------

void spline::setType(splineType type)
{
    this->type = type;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
