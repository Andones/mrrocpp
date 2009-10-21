
#if !defined (_TRAJECTORY_H_)
#define _TRAJECTORY_H_

#include <list>
#include <string>

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "ecp_mp/smooth_trajectory_pose.h"
#include "ecp_mp/smooth2_trajectory_pose.h"

#include <boost/serialization/utility.hpp>

namespace mrrocpp {
namespace ecp_mp {
namespace common {


class Trajectory
{
	public:
		Trajectory();
		Trajectory(const char *numOfPoses, std::string trajectoryName, const char *poseSpecification);
		Trajectory(const Trajectory &trajectory);
		~Trajectory();

		static int setValuesInArray(double arrayToFill[], const char *dataString);
		static lib::POSE_SPECIFICATION returnProperPS(const std::string & poseSpecification);
		static std::string toString(double valArr[], int length);
		static std::string toString(int numberOfPoses);
		static std::string toString(lib::POSE_SPECIFICATION ps);
		static std::string toString(lib::ROBOT_ENUM robot);

		static void writeTrajectoryToXmlFile(const char *fileName, lib::POSE_SPECIFICATION ps, std::list<ecp_mp::common::smooth_trajectory_pose> &poses);
		static void writeTrajectoryToXmlFile2(const char *fileName, lib::POSE_SPECIFICATION ps, std::list<ecp_mp::common::smooth2_trajectory_pose> &poses);//for smooth2
		void createNewPose();
		void createNewPose2();//for smooth2
		void addPoseToTrajectory();
		void addPoseToTrajectory2();//for smooth2

		void setTrjID(const char *trjID);
		const char * getTrjID() const;

		void setNumOfPoses(unsigned int numOfPoses);
		unsigned int getNumberOfPoses() const;

		void setPoseSpecification(const char *poseSpecification);
		lib::POSE_SPECIFICATION getPoseSpecification() const;

		void setStartVelocities(const char *startVelocities);
		double *getStartVelocities() const;

		void setEndVelocities(const char *endVelocities);
		double *getEndVelocities() const;

		void setVelocities(const char *Velocities);
		double *getVelocities() const;

		void setAccelerations(const char *accelerations);
		double *getAccelerations() const;

		void setVelocities2(const char *Velocities);//for smooth2
		double *getVelocities2() const;//for smooth2

		void setAccelerations2(const char *accelerations);//for smooth2
		double *getAccelerations2() const;//for smooth2

		void setCoordinates(const char *cCoordinates);//for smooth2
		double *getCoordinates() const;//for smooth2

		void setCoordinates2(const char *cCoordinates);//for smooth2
		double *getCoordinates2() const;//for smooth2

		void showTime();
		void showTime2();//for smooth2

		std::list<ecp_mp::common::smooth_trajectory_pose> & getPoses();
		std::list<ecp_mp::common::smooth2_trajectory_pose> & getPoses2();//for smooth2

	private:
		std::string trjID;
		unsigned int numOfPoses;
		lib::POSE_SPECIFICATION poseSpec;
		ecp_mp::common::smooth_trajectory_pose *actPose;
		ecp_mp::common::smooth2_trajectory_pose *actPose2;//for smooth2
		std::list<ecp_mp::common::smooth_trajectory_pose> trjPoses;
		std::list<ecp_mp::common::smooth2_trajectory_pose> trjPoses2;//for smooth2

		// boost serialization methods
		friend class boost::serialization::access;

		template<class Archive>
		    void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(trjID);
			ar & BOOST_SERIALIZATION_NVP(numOfPoses);
			ar & BOOST_SERIALIZATION_NVP(poseSpec);
			ar & BOOST_SERIALIZATION_NVP(trjPoses);
//			ar & BOOST_SERIALIZATION_NVP(trjPoses2);
		}
};

} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp



#endif
