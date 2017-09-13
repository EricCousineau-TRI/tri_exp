#include <gflags/gflags.h>

#include <pcl/io/pcd_io.h>

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

auto X_WW = Eigen::Isometry3d::Identity();

DEFINE_bool(with_perception, true,
            "Add perception for estimating book position.");

namespace tri_exp {

typedef PointCloudPerception<ColoredPointT, ColoredPointTNormal> PerceptionProc;

const std::string kPath =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

Isometry3d GetBookPose(PerceptionProc* pperception_proc,
                       pcl::PointCloud<ColoredPointT>::ConstPtr cloud_in) {
  PerceptionProc& perception_proc = *pperception_proc;
  // Need a copy...
  pcl::PointCloud<ColoredPointT>::Ptr cloud(new PointCloud<ColoredPointT>());
  pcl::copyPointCloud(*cloud_in, *cloud);

  perception_proc.OutlierRemoval(cloud);
  perception_proc.VisualizePointCloudDrake(cloud, X_WW, "Post OutlierRemoval");

  //cin.get();
  Eigen::Vector3f min_range;
  min_range << 0.1, -1.0, 0.6;
  Eigen::Vector3f max_range;
  max_range << 0.6, -0.2, 1;
  perception_proc.CutWithWorkSpaceConstraints(cloud, min_range, max_range);

  perception_proc.VisualizePointCloudDrake(cloud, X_WW, "Post Cut");\
  // perception_proc.VisualizePointCloud(cloud);

  pcl::io::savePCDFileASCII("pre_subtract.pcd", *cloud);

  // Get rid of the table.
  double thickness = 0.001;
  pcl::PointCloud<ColoredPointT>::Ptr removed(new PointCloud<ColoredPointT>());
  perception_proc.SubtractTable(cloud, thickness, removed);

  Eigen::Isometry3d X_WW = Eigen::Isometry3d::Identity();
  perception_proc.VisualizePointCloudDrake(removed, X_WW, "plane");
  perception_proc.VisualizePointCloudDrake(cloud, X_WW, "non_plane");

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
  PerceptionImpl(PerceptionProc* perception_proc)
      : PerceptionBase(Xtion::GetCameraIntrinsics()),
        perception_proc_(perception_proc) {
    // Use camera_info from simulation.
    DRAKE_DEMAND(perception_proc_ != nullptr);
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

    // Down-sample fused cloud.
    using T = ColoredPointT;
    pcl::VoxelGrid<T> grid;
    const double leaf_size = 0.001;
    std::cout << "Pre downsample: " << cloud_fused_->size() << std::endl;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud_fused_);
    grid.filter(*cloud_fused_);
    std::cout << "Post downsample: " << cloud_fused_->size() << std::endl;

    // perception_proc_->VisualizePointCloudDrake(cloud_W, X_WW, "READ");
    // perception_proc_->VisualizePointCloudDrake(cloud_fused_, X_WW, "FUSED");
  }

  Isometry3d EstimatePose() {
    DRAKE_ASSERT(!done_);
    Isometry3d out = GetBookPose(perception_proc_, cloud_fused_);
    done_ = true;
    return out;
  }

 private:
  bool done_{};
  PointCloud<ColoredPointT>::Ptr cloud_fused_;
  PerceptionProc* perception_proc_{};
};

}  // namespace tri_exp

using namespace tri_exp;

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  PerceptionProc perception_proc;

  return drake::examples::kuka_iiwa_arm::push_and_pick::DoMain(
      FLAGS_with_perception ?
         std::make_unique<PerceptionImpl>(&perception_proc) : nullptr);
}
