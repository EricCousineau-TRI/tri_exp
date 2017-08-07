#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

typedef pcl::PointXYZRGBNormal ColoredPointTNormal;
typedef pcl::PointNormal PointTNormal;

// Define a new point representation for combined xyzrgb,normal alignment.
template <typename T>
class GeometryAlignmentRepresentation : public 
  pcl::PointRepresentation <T> {
  using pcl::PointRepresentation<T>::nr_dimensions_;
 public:
  GeometryAlignmentRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const pcl::PointXYZRGBNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

// // Define a new point representation for combined xyz,normal alignment.
// class XYZAlignmentRepresentation : public 
//   pcl::PointRepresentation <pcl::PointXYZNormal> {
//   using pcl::PointRepresentation<pcl::PointXYZNormal>::nr_dimensions_;
//  public:
//   XYZAlignmentRepresentation ()
//   {
//     // Define the number of dimensions
//     nr_dimensions_ = 4;
//   }

//   // Override the copyToFloatArray method to define our feature vector
//   virtual void copyToFloatArray (const pcl::PointXYZRGBNormal &p, float * out) const
//   {
//     // < x, y, z, curvature >
//     out[0] = p.x;
//     out[1] = p.y;
//     out[2] = p.z;
//     out[3] = p.curvature;
//   }
// };

// This class implements methods for registration between point cloud pairs. 
class PointCloudPairRegistration {
 public:
 	PointCloudPairRegistration(){};
 	// Given a pair of point clouds with normals, register the second point cloud
 	// with respect to the first, and return a combined point cloud and the 
 	// relative tranformation.
 	template <typename T>
 	void RegisterPointCloudPair(const boost::shared_ptr<pcl::PointCloud<T>> src, 
 	    const boost::shared_ptr<pcl::PointCloud<T>> tgt,
 	    boost::shared_ptr<pcl::PointCloud<T>> combined,
 	    Eigen::Matrix4f* transform);
};

