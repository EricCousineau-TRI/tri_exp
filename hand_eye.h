#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni2_grabber.h>

#include <boost/thread/thread.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
//#include "drake/lcmt_iiwa_command.hpp"
//#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "perception.h"

// This class wraps over the openni grabber class to provide services to access
// the point cloud.
class OpenNiComm{
 public:
	OpenNiComm(){};
  void Run() {
    camera_interface_->start ();
  }

  void Stop() {
    camera_interface_->stop();
  }

 private:
 	void rgb_depth_image_cb_(const boost::shared_ptr<pcl::io::Image>& rgb_image,
      const boost::shared_ptr<pcl::io::DepthImage>& depth_image, 
      float reciprocalFocalLength);
 	void cloud_cb_ (const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> & cloud);
  pcl::Grabber* camera_interface_;

};

// This class implements hand-eye data coordination and data collection.
// Now, it seems that the eye is an OpenNi device and the hand is the end 
// effector of iiiwa robot.

const std::string kPath =
    "drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf";
const std::string kEEName = "iiwa_link_7";


class HandEye {
 public:
  HandEye(Eigen::Isometry3d tf_eye_wrt_hand, const RigidBodyTree<double>& robot, 
  		const std::string& end_effector_name)
      : tf_eye_wrt_hand_(tf_eye_wrt_hand),
      	robot_(robot),
      	end_effector_(*robot_.FindBody(end_effector_name)) {};
  // Set the transformation of the camera with respect to hand/tool frame.
  void SetEyeTransformationWrtHandFrame(Eigen::Isometry3d tf_eye_wrt_hand);

	// Get the pose of the camera with respect to robot base. 
	Eigen::Isometry3d GetCameraPoseWrtRobotBase();

	// Grab current point cloud from the openni camera (xtion pro) and save a 
	// copy for the class.	
	void GrabCurrentPointCloud(); 

	void SaveCurrentPointCloud(std::string file_name);
 private:
 	// Camera frame with respect to hand frame.
 	Eigen::Isometry3d tf_eye_wrt_hand_;
 	const RigidBodyTree<double>& robot_;
  const RigidBody<double>& end_effector_;

	bool flag_acquire_new_data;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pt_cloud;
	PointCloudPerception pcl_interface;

};