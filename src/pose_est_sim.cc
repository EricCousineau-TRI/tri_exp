#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"

#include "perception.h"

#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/manipulation/sensors/xtion.h"

#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/perception_base.h"
#include "drake/examples/kuka_iiwa_arm/dev/push_pick_place/push_and_pick_demo.h"

using pcl::PointCloud;
using drake::systems::sensors::RgbdCamera;
using drake::systems::sensors::CameraInfo;
using drake::systems::sensors::ImageDepth32F;
using drake::examples::kuka_iiwa_arm::push_and_pick::PerceptionBase;
using drake::manipulation::sensors::Xtion;

using Eigen::Isometry3d;
using Eigen::Matrix3Xf;

namespace tri_exp {

const std::string kPath =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

Isometry3d GetBookPose(pcl::PointCloud<ColoredPointT>::ConstPtr cloud_in) {
  PointCloudPerception<ColoredPointT, ColoredPointTNormal> perception_proc;

  // Need a copy...
  pcl::PointCloud<ColoredPointT>::Ptr cloud(new PointCloud<ColoredPointT>());
  pcl::copyPointCloud(*cloud_in, *cloud);

  perception_proc.OutlierRemoval(cloud);
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
  X_WO.translation() = center.cast<double>();
  X_WO.linear() << orientation.cast<double>();
  return X_WO;
}

class PerceptionImpl : public PerceptionBase {
 public:
  PerceptionImpl()
      : PerceptionBase(Xtion::GetCameraIntrinsics()) {
    // Use camera_info from simulation.
    done_ = false;
    cloud_fused_.reset(new PointCloud<ColoredPointT>());
  }

  void Update(double time,
              const ImageDepth32F& depth_image,
              const Isometry3d& X_WD) {
    DRAKE_ASSERT(!done_);

    // Fuse point cloud.
    Matrix3Xf points_D;
    RgbdCamera::ConvertDepthImageToPointCloud(depth_image, camera_info(),
      &points_D);
    Matrix3Xf points_W = X_WD.cast<float>() * points_D;

    pcl::PointCloud<ColoredPointT>::Ptr
        cloud_W(new pcl::PointCloud<ColoredPointT>());
    int n = points_W.cols();
    cloud_W->resize(n);

    // int k = 0;
    for (int i = 0; i < n; ++i) {
      auto& p = cloud_W->points[i];
      auto pt_W = points_W.col(i);
      p.x = pt_W[0];
      p.y = pt_W[1];
      p.z = pt_W[2];
      // const int u = i % width;
      // const int v = i / width;
      // cloud_W.point[i].color = img[k];
      // ++k;
    }

    *cloud_fused_ += *cloud_W;
  }

  Isometry3d EstimatePose() {
    DRAKE_ASSERT(!done_);
    Isometry3d out = GetBookPose(cloud_fused_);
    done_ = true;
    return out;
  }

 private:
  bool done_{};
  PointCloud<ColoredPointT>::Ptr cloud_fused_;
};

}  // namespace tri_exp

using namespace tri_exp;

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  return drake::examples::kuka_iiwa_arm::push_and_pick::DoMain(
      std::make_unique<PerceptionImpl>());
}