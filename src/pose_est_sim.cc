#include "perception.h"

#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/lcm_image_array_t_to_image.h"

#include "drake/examples/kuka_iiwa_arm/dev/push_and_pick/perception_base.h"

using namespace drake::systems::sensors;
using namespace drake::examples::kuka_iiwa_arm::push_and_pick;

namespace tri_exp {

const std::string kPath =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

Isometry3d GetBookPose(pcl::PointCloud<ColoredPointT>::Ptr cloud_in) {
  PointCloudPerception<ColoredPointT, ColoredPointTNormal> perception_proc;
  DrakeCameraInterface camera_interface;

  // Need a copy...
  pcl::PointCloud<ColoredPointT>::Ptr cloud(cloud_in);

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

  Isometry3d X_WO;
  X_WO.setIdentity();
  X_WO.translation() = center;
  X_WO.linear() << orientation;
}

class PerceptionImpl : public PerceptionBase {
 public:
  PerceptionImpl()
      : PerceptionBase(CameraInfo()) {
    // Use camera_info from simulation.
    cloud_fused_.reset(new PointCloud<ColoredPointT>());
  }

  void Update(double time,
              const ImageDepth32F& depth_image,
              const Isometry3d& X_WD) {
    // Fuse point cloud.
    RgbdCamera::ConvertDepthImageToPointCloud(depth_image, camera_info,
      &points_D);
    Matrix3Xf points_W = X_WD * points_D;

    pcl::PointCloud<ColoredPointT>::Ptr
        cloud_W(new pcl::PointCloud<ColoredPointT>());
    int n = points_W.cols();
    cloud_W->resize(n);

    // int k = 0;
    for (int i = 0; i < n; ++i) {
      auto& p = cloud_W.point[i];
      auto pt_W = points_W.col(i);
      p.x = pt_W[0];
      p.y = pt_W[1];
      p.z = pt_W[2];
      // const int u = i % width;
      // const int v = i / width;
      // cloud_W.point[i].color = img[k];
      // ++k;
    }

    *cloud_fused_ += cloud_W;
  }

  Isometry3d EstimatePose() {
    return GetBookPose(cloud_fused_);
  }

 private:
  PointCloud<ColoredPointT>::Ptr cloud_fused_;
};

}  // namespace tri_exp

using namespace tri_exp;

int main() {
  DoMain(new PerceptionImpl());

  return 0;
}
