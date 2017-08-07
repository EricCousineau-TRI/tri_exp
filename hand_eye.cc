#include "hand_eye.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

void HandEye::SetEyeTransformationWrtHandFrame(
	Eigen::Isometry3d tf_eye_wrt_hand) {

}

// Get the pose of the camera with respect to robot base. 
Eigen::Isometry3d GetCameraPoseWrtRobotBase(
	Eigen::Isometry3d tf_tool_wrt_base);

// Grab current point cloud from the openni camera (xtion pro) and save a 
// copy for the class.	
void GrabCurrentPointCloud(); 

void SaveCurrentPointCloud(std::string file_name);

}}}}