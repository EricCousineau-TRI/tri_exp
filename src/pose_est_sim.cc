#include "perception.h"

#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/lcm_image_array_t_to_image.h"

namespace tri_exp {

const std::string kPath =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

class DrakeCameraInterface {
 public:
  void UpdateReading(double time,
                     const ImageRgba8U& color_image,
                     const ImageDepth32F& depth_image) {
    state.time = time;
    state.color_image = color_image;
    state.depth_image = depth_image;
  }

  void GetCurrentPointCloud(pcl::PointCloud<ColoredPointT>::Ptr cloud) {
    // Get image.
    Matrix3Xf points = RgbdCamera::ToPointCloud(camera_info, state.depth_image);
    // Convert to PCL point cloud.
    const int n = points.cols();
    cloud.resize(n);
    Pixel* img = color_image.data();
    int k = 0;
    for (int i = 0; i < n; ++i) {
      cloud.point[i].xyz.EigenMap() = points.col(i);
      const int u = i % width;
      const int v = i / width;
      cloud.point[i].color = img[k];
      ++k;
    }
  }
 private:
  struct State {
    double time;
    ImageRgba8U color_image;
    ImageDepth32F depth_image;
  };
  State state_;
};


Isometry3d GetBookPose(pcl::PointCloud<ColoredPointT>::Ptr cloud_in) {
  PointCloudPerception<ColoredPointT, ColoredPointTNormal> perception_proc;
  DrakeCameraInterface camera_interface;

  // Need a copy...
  pcl::PointCLoud<ColoredPointT>::Ptr cloud(cloud_in);

  perception_proc.OutlierRemoval(cloud);

  Eigen::Affine3f camera_pose;
  camera_pose.matrix() = LcmPoseToPose(pose_msg).matrix().cast<float>();
  perception_proc.ApplyTransformToPointCloud(camera_pose, cloud);
  perception_proc.VisualizePointCloudDrake(cloud);

  //cin.get();
  Eigen::Vector3f min_range;
  min_range << 0.4, -0.5, -0.05;
  Eigen::Vector3f max_range;
  max_range << 0.95, 0.5, 0.3;
  perception_proc.CutWithWorkSpaceConstraints(cloud, min_range, max_range);
  // Get rid of the table.
  double thickness = 0.0125;
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

  return 0;
}

}  // namespace tri_exp
