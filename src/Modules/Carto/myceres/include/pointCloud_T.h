#ifndef _POINTCLOUD_H_
#define _POINTCLOUD_H_

#include <vector>

typedef struct s_xyz{
	double x;
	double y;
	double z;
}T_XYZ;
class PointCloud_T
{
public:
	inline int size() const
	{
		return xyz.size();
	}
	inline void set(double x,double y ,double z)
	{
		T_XYZ xyz1;
		xyz1.x = x;
		xyz1.y = y; 
		xyz1.z = z;
		xyz.push_back(xyz1);
	}
public:
	std::vector<T_XYZ> xyz; 
};

#endif
