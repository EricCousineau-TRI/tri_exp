#include "point_cloud_registration.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/octree/octree_search.h>
//#include <pcl/gpu/octree/octree.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <sys/time.h>

#include <cmath>



// This class provides the interface to several PCL algorithms.
// 1) Statistical removal of outliers.
// 2) Segmentation of the table.
// 3) Point cloud normal estimation.
// 4) Fuse multiple point clouds.
// 5) Apply transformation to point cloud.
// 6) Project the point cloud onto a plane


#pragma once

struct Camera
{
  Camera () : pose (Eigen::Affine3f::Identity()), fx(), fy(), img_height(),
                    img_width() {};
  Eigen::Affine3f pose;
  double fx;
  double fy;
  int img_height; // number of rows.
  int img_width;  // number of cols.
};

template <typename T, typename T2>
class PointCloudPerception {
  public:
  	PointCloudPerception() {};

  	void LoadPCDFile(std::string file_name,
  		boost::shared_ptr<pcl::PointCloud<T>> cloud);

    void DownSample(boost::shared_ptr<pcl::PointCloud<T>> cloud,
                    double leaf_size = 0.002);

    void CutWithWorkSpaceConstraints(boost::shared_ptr<pcl::PointCloud<T>> cloud,
        const Eigen::Vector3f & min_range, const Eigen::Vector3f& max_range);


    // Display the point cloud until 'q' key is pressed.
    void VisualizePointCloud(const boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud,
  		Eigen::Affine3f tf = Eigen::Affine3f::Identity());

    // Todo (Jiaji): refractor and redesign to fork a new thread just for visualization.
    void VisualizePointCloudAndNormal(const boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud,
      boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals,
      Eigen::Affine3f tf = Eigen::Affine3f::Identity());

    void SeparatePointsAndNormals(
      const boost::shared_ptr<pcl::PointCloud<T2>> points_and_normal, 
      boost::shared_ptr<pcl::PointCloud<T>> points, 
      boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals);

    //template <typename T2>
    void ApplyTransformToPointCloud(Eigen::Affine3f tf,
  		boost::shared_ptr<pcl::PointCloud<T>> cloud);

    void ApplyTransformToCombinedPointCloud(Eigen::Affine3f tf,
      boost::shared_ptr<pcl::PointCloud<T2>> cloud);


  	void OutlierRemoval(boost::shared_ptr<pcl::PointCloud<T>> cloud,
  		int num_neighbor = 50, double std_dev_threshold = 1.0);

  	// Get the plane coeffcients. ax + by + cz + d = 0, returned in vector4d.
    void FindPlane(const boost::shared_ptr<pcl::PointCloud<T>> cloud,
  		Eigen::Vector4d* coeffs_plane,  pcl::PointIndices::Ptr inliers,
  		double dist_threshold = 0.01);

  	// Points normals estimation. Assume the camera view is at world origin.
    // If the points are organized, we use integral image for speeding up.
    // Otherwise we use plain covariance matrix estimation with omp multi threading.
    void EstimateNormal(const boost::shared_ptr<pcl::PointCloud<T>> cloud,
      boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals,
      double radius = 0.01);

    // Projection point cloud onto a camera pose and generate opencv rgb image.
    // Note that the point cloud needs to be transformed to camera frame coordinate
    // first.
    cv::Mat ProjectColoredPointCloudToCameraImagePlane(
      const boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud,
      const Camera camera, int stride = 4);

  	// Fuse a pair of point clouds by aligning normal and curvature features.
    // A transformation will be applied to the second point cloud (tgt) to the
    // first point cloud (src).
    // Note that the two point cloud needs to be roughly aligned already with
    // reasonable amount of intersection.
    // The output will have normals.
    // Example T,T2 pair: <ColoredPointT, ColoredPointTNormal>, <PointT, PointTNormal>.
    //template <typename T2>
    void FusePointCloudPair(const boost::shared_ptr<pcl::PointCloud<T>> src,
        const boost::shared_ptr<pcl::PointCloud<T>> tgt,
        boost::shared_ptr<pcl::PointCloud<T2>> combined,
        Eigen::Matrix4f* transform) {
      // First, estimate normal and curvture for each point cloud.
      boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals_src(
          new pcl::PointCloud<pcl::Normal>);
      boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals_tgt(
          new pcl::PointCloud<pcl::Normal>);
      double radius_normal_est = 0.02;
      EstimateNormal(src, normals_src, radius_normal_est);
      EstimateNormal(tgt, normals_tgt, radius_normal_est);
      boost::shared_ptr<pcl::PointCloud<T2>> cloud_with_normal_src(
        new pcl::PointCloud<T2>);
      boost::shared_ptr<pcl::PointCloud<T2>> cloud_with_normal_tgt(
        new pcl::PointCloud<T2>);

      const pcl::PointCloud<T> & src_alias = *src;
      const pcl::PointCloud<pcl::Normal> & normals_src_alias = *normals_src;

      //ConcatPointAndNormal(src, normals_src, cloud_with_normal_src);
      //ConcatPointAndNormal(tgt, normals_tgt, cloud_with_normal_tgt);
      
      pcl::concatenateFields(*src, *normals_src, *cloud_with_normal_src);
      pcl::concatenateFields(*tgt, *normals_tgt, *cloud_with_normal_tgt);
      

      // for (int t = 0; t < 1000; ++t) {
      //   int i = rand() % src->size();
      //   std::cout << "Cloud point" << i << std::endl; 
      //   std::cout << src->points[i].x << std::endl;
      //   std::cout << tgt->points[i].x << std::endl;
      //   std::cout << normals_src->points[i].curvature << std::endl;
      //   std::cout << normals_tgt->points[i].curvature << std::endl;
      //   std::cout << cloud_with_normal_src->points[i].curvature << std::endl;
      //   std::cout << cloud_with_normal_tgt->points[i].curvature << std::endl;
      // }

      PointCloudPairRegistration reg;
      reg.RegisterPointCloudPair<T2>(cloud_with_normal_src, cloud_with_normal_tgt,
                                    combined, transform);
    }

    // //template <typename T2>
    // void ConcatPointAndNormal(const boost::shared_ptr<pcl::PointCloud<T>> cloud, 
    //   const boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals,
    //   boost::shared_ptr<pcl::PointCloud<T2>> points_normals) {

    //   assert(cloud->size() == normals->size());
    //   points_normals->reserve(cloud->size());
    //   for (int i = 0; i < cloud->size(); ++i) {
    //     T2 pt;
    //     pt.x = cloud->points[i].x;
    //     pt.y = cloud->points[i].y;
    //     pt.z = cloud->points[i].z;
    //     bool has_color = pcl::traits::has_field<T, pcl::fields::rgba>::value; 
    //     if (has_color) {
    //       pt.rgb = cloud->points[i].rgb;
    //       // pt.r = points.points[i].r;
    //       // pt.g = points.points[i].g;
    //       // pt.b = points.points[i].b;
    //       // pt.a = points.points[i].a;
    //     }
    //     pt.normal_x = normals->points[i].normal_x;  
    //     pt.normal_y = normals->points[i].normal_y;
    //     pt.normal_z = normals->points[i].normal_z;
    //     pt.curvature = normals->points[i].curvature;
    //     points_normals->points[i]= pt;
    //   }
    // }

    // Fuse multiple point clouds seqentially. Every consecutive pairs are fused
    // and added to the combined one.
    //template <typename T2>
    void FuseMultiPointClouds(
        const std::vector< boost::shared_ptr<pcl::PointCloud<T>> > point_clouds,
        boost::shared_ptr<pcl::PointCloud<T2>> combined_cloud) {

      Eigen::Affine3f global_tf_affine = Eigen::Affine3f::Identity();
      //Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
      for (unsigned i = 0; i < point_clouds.size(); ++i) {
        if (i == 1) continue;
        boost::shared_ptr<pcl::PointCloud<T2>> tmp_combined_cloud(new pcl::PointCloud<T2>);
        Eigen::Matrix4f relative_transform;
        //std::cout << point_clouds[i-1]->size() << std::endl;
        FusePointCloudPair(point_clouds[1], point_clouds[i],  
            tmp_combined_cloud, &relative_transform);
        // global_tf_affine.matrix() = global_tf_affine.matrix() * relative_transform;
        // ApplyTransformToCombinedPointCloud(global_tf_affine, tmp_combined_cloud);
        // *combined_cloud += *tmp_combined_cloud;
        std::string f_name = "test" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileASCII(f_name, *tmp_combined_cloud);
        *combined_cloud += *tmp_combined_cloud;
      }
    }

  private:

};
