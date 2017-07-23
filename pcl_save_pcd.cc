#include <pcl/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <memory>
#include <sys/time.h>
#include <iostream>

#include <cmath>


class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () {
      //viewer ("PCL OpenNI Viewer") 
      pcl_viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("test");
      pcl_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_cloud2");
      pcl_viewer->addCoordinateSystem (1.0);
      pcl_viewer->initCameraParameters ();
     }
     
   
     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {
        static unsigned count = 0;
        static double last = pcl::getTime ();
        if (++count == 30)
        {

          double now = pcl::getTime ();
          std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
          count = 0;
          last = now;
          if (flag_initial_cloud) {
            pcl_viewer->setBackgroundColor (0, 0, 0);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
            pcl_viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample_cloud2");
            pcl_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_cloud2");
            pcl_viewer->addCoordinateSystem (0.2);
            pcl_viewer->initCameraParameters ();
            flag_initial_cloud = false;
          }
          else {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cp(new pcl::PointCloud<pcl::PointXYZRGBA>);
            copyPointCloud(*cloud, *cloud_cp);
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.01);  

            seg.setInputCloud(cloud_cp);
            seg.segment(*inliers, *coefficients);

            for (int i = 0; i < inliers->indices.size(); ++i) {
              cloud_cp->points[inliers->indices[i]].r = 255;
              cloud_cp->points[inliers->indices[i]].g = 0; 
              cloud_cp->points[inliers->indices[i]].b = 0;
              
            }
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_cp);
            pcl_viewer->updatePointCloud<pcl::PointXYZRGBA> (cloud_cp, rgb, "sample_cloud2");

            std::cout  << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

            std::cin.get();
            // Store the point cloud. 
            pcl::io::savePCDFileASCII("test_pcd.pcd", *cloud);
            std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;

          }
        }
      }


     void run ()
     {
       pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> g =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback(g);
       //interface->registerCallback(g);

       interface->start ();

        while (!pcl_viewer->wasStopped())
        {
          pcl_viewer->spinOnce(100);
          boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

       interface->stop ();
     }

     boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer;
     bool flag_initial_cloud;
 };

 int main(int argc, char** argv) {
  SimpleOpenNIViewer viewer;
  viewer.run();

 }