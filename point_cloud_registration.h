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

typedef pcl::PointXYZRGB ColoredPointT;
typedef pcl::PointXYZ PointT;
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
  virtual void copyToFloatArray (const T &p, float * out) const
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
 	void RegisterPointCloudPair(
 		const boost::shared_ptr<pcl::PointCloud<T>> src, 
    const boost::shared_ptr<pcl::PointCloud<T>> tgt,
    boost::shared_ptr<pcl::PointCloud<T>> output,
    Eigen::Matrix4f* final_transform) {

  GeometryAlignmentRepresentation<T> point_representation;
  // Weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  pcl::IterativeClosestPoint<T, T> reg;
  reg.setTransformationEpsilon (1e-4);
  // Set the maximum distance between two correspondences (src<->tgt) to some initial value.
  // Note: adjust this based on the size of your datasets
  double init_correspondence_dist = 0.1;
  reg.setMaxCorrespondenceDistance (init_correspondence_dist);  
  // Set the point representation
  reg.setPointRepresentation(
  	boost::make_shared<const GeometryAlignmentRepresentation<T>> (point_representation));

  boost::shared_ptr<pcl::PointCloud<T>> points_with_normals_src(new pcl::PointCloud<T>);
  boost::shared_ptr<pcl::PointCloud<T>> points_with_normals_tgt(new pcl::PointCloud<T>);
  
  pcl::copyPointCloud(*src, *points_with_normals_src);
  //pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

  reg.setInputSource (src);
  reg.setInputTarget (tgt);



   // for (int t = 0; t < 1000; ++t) {
   //      int i = rand() % points_with_normals_tgt->size();
   //      std::cout << "Cloud point 2 :" << i << std::endl; 
   //      std::cout << points_with_normals_src->points[i].x << std::endl;
   //      std::cout << points_with_normals_tgt->points[i].x << std::endl;
   //      if (!pcl::isFinite(points_with_normals_tgt->points[i]))
   //      	std::cout << "Not finite" << std::endl;
   //      std::cout << points_with_normals_src->points[i].curvature << std::endl;
   //      std::cout << points_with_normals_tgt->points[i].curvature << std::endl;
   //    }


  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity ();
  Eigen::Matrix4f prev;

 	int num_iters = 3;
  reg.setMaximumIterations (num_iters);
  
 	//reg.setInputSource (src);
  //pcl::PointCloud<T> tmp;
  reg.align (*points_with_normals_src, Ti);

  Ti = reg.getFinalTransformation () * Ti;
  std::cout << Ti << std::endl;

  int num_extra_iters = 2;
  for (int i = 0; i < 1 + num_extra_iters; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);
    std::cout << points_with_normals_src->size() << std::endl;
    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*points_with_normals_src);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < 
    	reg.getTransformationEpsilon ()) {
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance ()* 0.5);
    }
    prev = reg.getLastIncrementalTransformation ();
  }

  // // Get the transformation from target to source
  Eigen::Matrix4f targetToSource = Ti.inverse();

  // Transform target onto source frame
  pcl::transformPointCloud (*tgt, *output, targetToSource);

  //add the source to the transformed target
  *output += *src;
  *final_transform = targetToSource;  	
 	}
};

