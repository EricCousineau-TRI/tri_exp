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
    nr_dimensions_ = 3;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const T &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    // out[3] = p.curvature;
    // out[4] = p.normal_x;
   	// out[5] = p.normal_x;
    // out[6] = p.normal_x;
     
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


  // Run the same optimization in a loop and visualize the results
  int round_seeds = 1;
  double least_error = 1e+9;
  int ct_round = 0;
  double range_sample_angle =  0 * M_PI / 24.0;
  Eigen::Matrix4f targetToSource;
  srand(-1);
  boost::shared_ptr<pcl::PointCloud<T>> points_with_normals_src(new pcl::PointCloud<T>);
  while (ct_round < round_seeds) {
  	double angle_x = ((rand() % 1000) - 500) / 500.0 * range_sample_angle;
  	double angle_y = ((rand() % 1000) - 500) / 500.0 * range_sample_angle;
  	double angle_z = ((rand() % 1000) - 500) / 500.0 * range_sample_angle;

  	Eigen::Matrix3f mat = (Eigen::AngleAxisf(angle_x, Eigen::Vector3f::UnitX())
  			* Eigen::AngleAxisf(angle_y,  Eigen::Vector3f::UnitY())
  			* Eigen::AngleAxisf(angle_z, Eigen::Vector3f::UnitZ())).toRotationMatrix();
	  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity ();
	  
	  Ti.topLeftCorner(3,3) = mat;
	  //Ti(2,3) = -0.02;
	  std::cout << "Initial transformation" << std::endl;
	  // Eigen::Vector4f centroid_src, centroid_tgt;
	  // pcl::compute3DCentroid(src, &centroid_src);
	  // pcl::compute3DCentroid(tgt, &centroid_tgt);
	  // std::cout << centroid_src << std::endl;
	  // std::cout << centroid_tgt << std::endl;
	  std::cout << Ti << std::endl;
	  targetToSource = Ti.inverse();
	  
	  Eigen::Matrix4f prev;

	  GeometryAlignmentRepresentation<T> point_representation;
	  // Weight the 'curvature' dimension so that it is balanced against x, y, and z
	  float alpha[3] = {1.0, 1.0, 1.0};
	  //float alpha[7] = {1.0, 1.0, 1.0, .001, .001, .001, .001};
	  point_representation.setRescaleValues (alpha);

	  pcl::IterativeClosestPoint<T, T> reg;
	  reg.setTransformationEpsilon (1e-5);
	  // Set the maximum distance between two correspondences (src<->tgt) to some initial value.
	  // Note: adjust this based on the size of your datasets
	  double init_correspondence_dist = 0.05;
	  reg.setMaxCorrespondenceDistance (init_correspondence_dist);  
	  // Set the point representation
	  reg.setPointRepresentation(
	  	boost::make_shared<const GeometryAlignmentRepresentation<T>> (point_representation));

	  
	  
	  pcl::copyPointCloud(*src, *points_with_normals_src);

	  reg.setInputSource (src);
	  reg.setInputTarget (tgt);

	 	int num_iters = 20;
	  reg.setMaximumIterations (num_iters);
	  
	  pcl::PointCloud<T> tmp;
	  reg.align (*points_with_normals_src, Ti);

	  Ti = reg.getFinalTransformation () * Ti;
	  //std::cout << Ti << std::endl;

	  int num_extra_iters = 20;
	  double final_correspondence_dist = 0.001;
	  double decr_dist = 
	  	(init_correspondence_dist - final_correspondence_dist)  / num_extra_iters;
	  for (int i = 0; i < num_extra_iters; ++i)
	  {
	    PCL_INFO ("Iteration Nr. %d.\n", i);
	    //std::cout << points_with_normals_src->size() << std::endl;
	    // Estimate
	    reg.setInputSource (points_with_normals_src);
	    reg.align (*points_with_normals_src);

			//accumulate transformation between each Iteration
	    Ti = reg.getFinalTransformation () * Ti;
	    //std::cout << Ti << std::endl;
			//if the difference between this transformation and the previous one
			//is smaller than the threshold, refine the process by reducing
			//the maximal correspondence distance
	    //reg.setMaxCorrespondenceDistance (init_correspondence_dist - decr_dist * (i + 1));	
	    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < 
	    	reg.getTransformationEpsilon ()) {
	      reg.setMaxCorrespondenceDistance (init_correspondence_dist - decr_dist * (i + 1));
	    }
	    prev = reg.getLastIncrementalTransformation ();
	    std::cout << "has converged:" << reg.hasConverged() << 
	      " score: " << reg.getFitnessScore() << std::endl; 
	  }
	  if (reg.getFitnessScore() < least_error) {
	  	// Get the transformation from target to source
		  targetToSource = Ti.inverse();
		  least_error = reg.getFitnessScore();
	  	std::cout << targetToSource << std::endl;
			std::cout << least_error << std::endl;
	  }
	    ct_round++; 
 	}
 		// std::cout << "Best among different ICs" << std::endl;
	// std::cout << targetToSource << std::endl;
	// std::cout << least_error << std::endl;
  // Transform target onto source frame
 	pcl::transformPointCloud (*tgt, *output, targetToSource);
  //add the source to the transformed target
  *output += *src;
  // *output += *points_with_normals_src;
  // *output += *tgt;
  *final_transform = targetToSource;  	
 }
};

