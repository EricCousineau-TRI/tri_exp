#include "point_cloud_registration.h"

template<typename T>
void PointCloudPairRegistration::RegisterPointCloudPair(
	const boost::shared_ptr<pcl::PointCloud<T>> src, 
    const boost::shared_ptr<pcl::PointCloud<T>> tgt,
    boost::shared_ptr<pcl::PointCloud<T>> output,
    Eigen::Matrix4f* final_transform) {

  GeometryAlignmentRepresentation<T> point_representation;
  // Weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  pcl::IterativeClosestPointNonLinear<T, T> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to some initial value.
  // Note: adjust this based on the size of your datasets
  double init_correspondence_dist = 0.05;
  reg.setMaxCorrespondenceDistance (init_correspondence_dist);  
  // Set the point representation
  reg.setPointRepresentation(
  	boost::make_shared<const GeometryAlignmentRepresentation<T>> (point_representation));

  boost::shared_ptr<pcl::PointCloud<T>> points_with_normals_src(new pcl::PointCloud<T>);
  boost::shared_ptr<pcl::PointCloud<T>> points_with_normals_tgt(new pcl::PointCloud<T>);
  
  pcl::copyPointCloud(*src, *points_with_normals_src);
  pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity ();
  Eigen::Matrix4f prev;

 	int num_iters = 20;
  reg.setMaximumIterations (num_iters);
  
  int num_extra_iters = 2;
  for (int i = 0; i < 1 + num_extra_iters; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

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
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.01);
    }
    prev = reg.getLastIncrementalTransformation ();
  }

  // Get the transformation from target to source
  Eigen::Matrix4f targetToSource = Ti.inverse();

  // Transform target onto source frame
  pcl::transformPointCloud (*tgt, *output, targetToSource);

  //add the source to the transformed target
  *output += *src;
  final_transform = targetToSource;  	
}