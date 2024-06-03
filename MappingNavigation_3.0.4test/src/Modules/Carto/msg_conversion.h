
#ifndef MSG_CONVERSION_H
#define MSG_CONVERSION_H

//#include "port.h"
#include "mytime.h"

//#include "landmark_data.h"
#include "point_cloud.h"
#include "rigid_transform.h"


#include <vector>

#include "rigid_transform.h"

using namespace std;



namespace sensor_msgs{



struct Point
{
	float x;
	float y;
	float z;
};
struct Quaternion 
{
	float x;
	float y;
	float z;
	float w;
};
struct Pose
{
	Point position;
	Quaternion orientation;
};

struct Header{

	uint32_t seq;
	common::Time stamp;
	std::string frame_id;
};

struct PoseWithCovariance
{
	Pose pose;
	// Row-major representation of the 6x6 covariance matrix
	// The orientation parameters use a fixed-axis representation.
	// In order, the parameters are:
	// (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
	float covariance[36];
};

struct PoseWithCovarianceStamped
{

	Header header;
	PoseWithCovariance pose;
};

struct Vector3
{
	float x;
	float y;
	float z;
};
struct Twist
{
	Vector3  linear;
	Vector3  angular;
};
struct TwistWithCovariance
{
	Twist twist;
	float covariance[36];
};


struct TwistStamped
{
	Header header;
	Twist twist;


};



struct TwistWithCovarianceStamped
{
	Header header;
	TwistWithCovariance twist;
};


struct Odometry{

	Header header;
	std::string child_frame_id;
	PoseWithCovariance pose;
	TwistWithCovariance twist;

};


struct OdometryProto{

	double stamp;  //sec
 	float x;
	float y;
	float theta;
};

struct Imu
{
	Header header;

	Quaternion orientation;
	float  orientation_covariance[9] ;// # Row major about x, y, z axes

	Vector3 angular_velocity;
	float angular_velocity_covariance[9];  //# Row major about x, y, z axes

	Vector3 linear_acceleration;
	float linear_acceleration_covariance[9]; //# Row major x, y z 
};



}


namespace sensor_msgs{



	struct LaserScan{

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Header header;            // timestamp in the header is the acquisition time of 
		                     // the first ray in the scan.
		                     //
		                     // in frame frame_id, angles are measured around 
		                     // the positive Z axis (counterclockwise, if Z is up)
		                     // with zero angle being forward along the x axis
		                     
	float angle_min;        // start angle of the scan [rad]
	float angle_max;        // end angle of the scan [rad]
	float angle_increment;  // angular distance between measurements [rad]
	float time_increment;   // time between measurements [seconds] - if your scanner
                         // is moving, this will be used in interpolating position
                         // of 3d points
	float scan_time;        // time between scans [seconds]

	float range_min;        // minimum range value [m]
	float range_max ;       // maximum range value [m]

	vector<float> ranges;         // range data [m] (Note: values < range_min or > range_max should be discarded)
	vector<float> intensities;    // intensity data [device-specific units].  If your
		                     // device does not provide intensities, please leave
		                     // the array empty.
		               

	std::string sensor_id;
	Eigen::Matrix<double, 3, 1> laser_tf ;

};
}






#endif  // MSG_CONVERSION_H
