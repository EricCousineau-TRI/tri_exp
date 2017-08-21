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
  double width_book = 0.18;
  double height_book = 0.23;
  double half_finger_thickness = 0.012;
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
  double duration = 2.0;
  robot_comm.MoveToJointPositionDegrees(joint_target_up, duration);

  OpenNiComm camera_interface;
	boost::shared_ptr<pcl::PointCloud<ColoredPointT>> cloud(
			new pcl::PointCloud<ColoredPointT>);
	camera_interface.GetCurrentPointCloud(cloud);
	perception_proc.OutlierRemoval(cloud);

 	Eigen::Isometry3d camera_pose_iso = robot_comm.GetCartesianPose() * tf_camera_wrt_hand;
	Eigen::Affine3f camera_pose;
	camera_pose.matrix() = camera_pose_iso.matrix().cast<float>();
	perception_proc.ApplyTransformToPointCloud(camera_pose, cloud);

	Eigen::Vector3f min_range;
	min_range << 0.4, -0.5, -0.2;
	Eigen::Vector3f max_range;
	max_range << 0.95, 0.5, 0.5;
	perception_proc.CutWithWorkSpaceConstraints(cloud, min_range, max_range);
	// Get rid of the table.
  double thickness = 0.01;
  perception_proc.SubtractTable(cloud, thickness);

	Eigen::Vector3f center, top_corner, lower_corner;
	Eigen::Matrix3f orientation;
	double cover_ratio = 0.95;
  perception_proc.FindBoundingBox(cloud, &center, &top_corner, 
  	&lower_corner, &orientation, cover_ratio);
  double yaw_angle_radian = atan2(orientation(1,0), orientation(0,0));
  std::cout << "Center point bbox: " << center.transpose() << std::endl;
  std::cout << "Top right corner: " << top_corner.transpose() << std::endl;
  std::cout << "Lower left corner: " << lower_corner.transpose() << std::endl;
  std::cout << "Orientation: " << orientation << std::endl;
  std::cout << "Yaw angle: " << yaw_angle_radian * 180 / M_PI << std::endl;

  Eigen::VectorXd joint_buffer_above(7);
  joint_buffer_above << 1.75373,11.7494,-2.09317,-86.2383,0.42104,82.5027,21.6225;
  robot_comm.MoveToJointPositionDegrees(joint_buffer_above, duration);

  double z_above = 0.05;
  Eigen::Isometry3d pose_approach = Eigen::Isometry3d::Identity();
  pose_approach.linear() = pose_approach.linear() 
  	* Eigen::AngleAxis<double>(yaw_angle_radian, Eigen::Vector3d::UnitZ());
  pose_approach.translation() = center.cast<double>();
  pose_approach.translation()(2) += z_above; 
  robot_comm.MoveToCartesianPose(pose_approach, 2.0);

  // Move down.
  double z_penetration = 0.01;
  Eigen::Isometry3d pose_contact = pose_approach;
  pose_contact.translation() = center.cast<double>();
  pose_contact.translation()(2) += - z_penetration;
  robot_comm.MoveToCartesianPose(pose_contact, 2.0, 0, 0);

  // Twist to align with base frame.
  Eigen::Isometry3d pose_contact_align = Eigen::Isometry3d::Identity();
  pose_contact_align.translation() = center.cast<double>();
  pose_contact_align.translation()(2) += - z_penetration;
  
  robot_comm.MoveToCartesianPose(pose_contact_align, 3.0, 60, 0);

  double push_dist = 0.08;
  double lift_up_duration = 1.0;
  double move_down_duration = 1.4;
  double up_in_air_duration = 1.0;
  double push_align_duration = 1.0;
  // // Move Up. 
  Eigen::Isometry3d pose_push = pose_contact_align;
  // pose_push.translation()(2) += 0.2;
  // robot_comm.MoveToCartesianPose(pose_push, lift_up_duration);
  // // Move to one edge 
  // pose_push.translation()(0) -= 0.15;
  // pose_push.linear() = Eigen::Matrix3d::Identity();
  // robot_comm.MoveToCartesianPose(pose_push, up_in_air_duration);
  // //Move down. 
  // pose_push.translation()(2) = center(2) - 0.02;
  // robot_comm.MoveToCartesianPose(pose_push, move_down_duration);
  // // Move towards/away from the book.
  // pose_push.translation()(0) += push_dist;
  // robot_comm.MoveToCartesianPose(pose_push, push_align_duration);
  // // Orthogonal push.
  // // Move up. 
  // pose_push.translation()(2) += 0.2;
  // double dist_back_off = 0.05;
  // pose_push.translation()(0) -= dist_back_off;
  // robot_comm.MoveToCartesianPose(pose_push, lift_up_duration);
  // pose_push.linear() = pose_push.linear() *  
  // 		Eigen::AngleAxis<double>(M_PI / 2.0, Eigen::Vector3d::UnitZ());
  // pose_push.translation()(0) += (height_book / 2.0 + half_finger_thickness + dist_back_off);
  // pose_push.translation()(1) -= 0.15;
  // robot_comm.MoveToCartesianPose(pose_push, up_in_air_duration + 0.5);
  // // Move Down.
  // pose_push.translation()(2) = center(2) - 0.02;
  // robot_comm.MoveToCartesianPose(pose_push, move_down_duration);
  // // Move towards the book.
  // pose_push.translation()(1) += push_dist; 
  // robot_comm.MoveToCartesianPose(pose_push, push_align_duration);
  // Now the book should be exactly localized. 
  // pose_push.translation()(2) += 0.2;
  // robot_comm.MoveToCartesianPose(pose_push, lift_up_duration);
  // pose_push.translation()(1) += (width_book / 2.0 + half_finger_thickness);
  // pose_push.translation()(2) = (center(2) - z_penetration);
  // robot_comm.MoveToCartesianPose(pose_push, move_down_duration);
 
  //----------------------------- Single Push. 
  pose_push.translation()(2) += 0.15;
  robot_comm.MoveToCartesianPose(pose_push, lift_up_duration);
  pose_push.translation()(1) -= (width_book / 2.0 + half_finger_thickness + 0.02);
  pose_push.linear() = pose_push.linear() *  
 		Eigen::AngleAxis<double>(M_PI / 2.0, Eigen::Vector3d::UnitZ());
  robot_comm.MoveToCartesianPose(pose_push, up_in_air_duration);
  pose_push.translation()(2) = center(2) - 0.02;
  robot_comm.MoveToCartesianPose(pose_push, move_down_duration);
  pose_push.translation()(1) += 0.05;
  robot_comm.MoveToCartesianPose(pose_push, push_align_duration);

 	// Now the book should be exactly localized. 
  pose_push.translation()(2) += 0.2;
  robot_comm.MoveToCartesianPose(pose_push, lift_up_duration);
  pose_push.translation()(1) += (width_book / 2.0 + half_finger_thickness);
  pose_push.translation()(2) = (center(2) - z_penetration);
  robot_comm.MoveToCartesianPose(pose_push, move_down_duration); 

  // Move to fixed pose at the edge of the table.
  double edge_x = 0.48;
  double edge_y = 0.345;
  //double edge_y = 0.365;
  //double edge_y = 0.345 - (width_book/2 + half_finger_thickness);
  //Eigen::Isometry3d pose_table_edge = pose_contact_align;
  Eigen::Isometry3d pose_table_edge = pose_push;
  //Eigen::Isometry3d pose_table_edge = pose_contact_align;
  pose_table_edge.translation()(0) = edge_x;
  pose_table_edge.translation()(1) = edge_y;
  robot_comm.MoveToCartesianPose(pose_table_edge, 4.0, 40, 0);

  // Leave contact.
  Eigen::Isometry3d pose_table_edge_above = pose_table_edge;
  //pose_table_edge_above.translation()(1) += further_away_y;
  pose_table_edge_above.translation()(2) += 0.15;
  robot_comm.MoveToCartesianPose(pose_table_edge_above, 1.0);
  
  double further_away_y = 0.055;
  //double further_away_y = 0.05;
  //double further_away_y = 0.05 + (width_book/2 + half_finger_thickness);
  pose_table_edge_above.linear() = Eigen::Matrix3d::Identity();
  pose_table_edge_above.translation()(1) += further_away_y;
  robot_comm.MoveToCartesianPose(pose_table_edge_above, 2.0);

  // Rotate gripper and moves down to pregrasp.
  double roll_angle_radian = - 35.0 / 180 * M_PI;
  double z_grasp_height = -0.00;
  Eigen::Isometry3d pose_table_edge_pregrasp = pose_table_edge_above;
  pose_table_edge_pregrasp.linear() =  pose_table_edge_pregrasp.linear() * 
  		Eigen::AngleAxis<double>(roll_angle_radian, Eigen::Vector3d::UnitX());
  pose_table_edge_pregrasp.translation()(2) = z_grasp_height;
  robot_comm.MoveToCartesianPose(pose_table_edge_pregrasp, 2.0);
  // Closes gripper.
  robot_comm.CloseGripper();

  // 
  double book_weight = 0.0 * 9.8; 
  // Lift up the robot.
  Eigen::Isometry3d pose_table_edge_grasp_lift = pose_table_edge_pregrasp;
  double z_lift = 0.2;
  double y_inward = -0.20;
  pose_table_edge_grasp_lift.translation()(2) += z_lift;
  pose_table_edge_grasp_lift.translation()(1) += y_inward;
  robot_comm.MoveToCartesianPose(pose_table_edge_grasp_lift, 5.0, -book_weight);

  // Move the robot back to table.
  Eigen::Isometry3d pose_drop = Eigen::Isometry3d::Identity();
  pose_drop.translation() = Eigen::Vector3d(edge_x, 0, 0.4);
  robot_comm.MoveToCartesianPose(pose_drop, 3.0, -book_weight);

  robot_comm.OpenGripper();

  while (true) {};
	return 0;
}