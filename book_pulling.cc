#include "robot_comm.h"
#include "perception.h"
#include "openni_comm.h"

const Eigen::Isometry3d tf_hand_to_ee(
    Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.2384)) *
    Eigen::AngleAxis<double>(-22. / 180. * M_PI, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d::UnitY()));

const std::string kPath =
    "/home/user/drake_siyuan/drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";



int main() {
	RigidBodyTree<double> tree;
  	drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kPath, drake::multibody::joints::kFixed, nullptr, &tree);
	RigidBodyFrame<double> hand_frame("tool", tree.FindBody(drake::jjz::kEEName),
  		tf_hand_to_ee);
	RobotComm robot_comm(tree, hand_frame);

	Eigen::Isometry3d tf_camera_wrt_ee;
  tf_camera_wrt_ee.matrix() << 
    0.3994,   -0.9168,   -0.0015,   -0.0446,
    0.9163,    0.3992,   -0.0317,    0.0011,
    0.0297,    0.0113,    0.9995,    0.1121,
         0,         0,         0,    1.0000;

  Eigen::Isometry3d tf_camera_wrt_hand = tf_hand_to_ee.inverse() * tf_camera_wrt_ee;

	PointCloudPerception<ColoredPointT, ColoredPointTNormal> perception_proc;

 	Eigen::VectorXd joint_target_up(7);
  joint_target_up << -49.9, -35.1 , 4.9, -86.3, 22.6, 113.0, 6.1;
  joint_target_up = joint_target_up / 180 * M_PI;
  double duration = 2.0;
  robot_comm.MoveToJointPosition(joint_target_up, duration);

  OpenNiComm camera_interface;
	boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud(
			new pcl::PointCloud<ColoredPointT>);
	camera_interface.GetCurrentPointCloud(cloud);
	perception_proc.OutlierRemoval(cloud);

 	Eigen::Isometry3d camera_pose_iso = robot_comm.GetCartesianPosition() * tf_camera_wrt_hand;
	Eigen::Affine3f camera_pose;
	camera_pose.matrix() = camera_pose_iso.matrix().cast<float>();
	perception_proc.ApplyTransformToPointCloud(camera_pose, cloud);

  // Get rid of the table.
  double thickness = 0.01;
  perception_proc.SubtractTable(cloud, thickness);

	Eigen::Vector3f min_range;
	min_range << 0.4, -0.45, -0.2;
	Eigen::Vector3f max_range;
	max_range << 0.9, 0.45, 0.5;
	perception_proc.CutWithWorkSpaceConstraints(cloud, min_range, max_range);

	Eigen::Vector3f center, top_right_corner, lower_left_corner;
	Eigen::Matrix3f orientation;
	double cover_ratio = 0.95;
  perception_proc.FindBoundingBox(cloud, &center, &top_right_corner, 
  	&lower_left_corner, &orientation, cover_ratio);
  std::cout << "Center point bbox: " << center.transpose() << std::endl;
  std::cout << "Top right corner: " << top_right_corner.transpose() << std::endl;
  std::cout << "Lower left corner: " << lower_left_corner.transpose() << std::endl;
  std::cout << "Orientation: " << orientation << std::endl;
  std::cout << "Yaw angle: " << 
  		(atan2(orientation(1,0), orientation(0,0)) * 180 / M_PI) << std::endl;


	return 0;
}