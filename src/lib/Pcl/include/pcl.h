#ifndef __MINI_PCL
#define __MINI_PCL

#include <vector>

namespace pcl
{
	struct PointXYZ
	{
		float x;
		float y;
		float z;

		PointXYZ(float _x, float _y, float _z)
		{
			x = _x;
			y = _y;
			z = _z;
		}

		PointXYZ() 
		{
			x = y = z = 0;
		}
	};

	template <typename PointT> 
	class PointCloud
	{
	public:
		typedef std::vector<PointT> VectorType;
		typedef typename VectorType::const_iterator const_iterator;

		//std::vector<PointT> points;
		VectorType points;

	public:

		inline void	clear()
		{
			points.clear();
		}

		inline void	push_back(const PointT& pt)
		{
			points.push_back(pt);
		}

		inline size_t size() const { return (points.size()); }

		inline PointCloud& operator += (const PointCloud& rhs)
		{
			// Make the resultant point cloud take the newest stamp
			size_t nr_points = points.size();
			points.resize(nr_points + rhs.points.size());

			for (size_t i = nr_points; i < points.size(); ++i)
				points[i] = rhs.points[i - nr_points];

			return *this;
		}
	};

}
#endif
