#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"

// PCL (openni grabber) and april tag related stuffs.
#include <pcl/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sys/time.h>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>
#include <AprilTags/TagFamily.h>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";

constexpr double kUninitTime = -1.0;
 const std::string kPath =
    "drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf";
const std::string kEEName = "iiwa_link_ee";
const Isometry3<double> kBaseOffset = Isometry3<double>::Identity();


// Example usage:
//AprilTagPerception test;
//  test.Run();
//  boost::this_thread::sleep (boost::posix_time::seconds (5));
//  test.Stop();

class AprilTagPerception {
 public:
  // Default is 36h11 family with A4 paper print size.
  AprilTagPerception() : tag_code_(AprilTags::tagCodes36h11) {
    tag_size_ = 0.166;
    // Got the information about intrinsics from ROS.
    // In terminal run: roslaunch openni2_launch openni2.launch.
    // Then rostopic echo /camera/rgb/camera_info
    // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    // D: [0.0, 0.0, 0.0, 0.0, 0.0]
    // K: [570.3422241210938, 0.0, 319.5,
    // 0.0, 570.3422241210938, 239.5,
    // 0.0, 0.0, 1.0]
    // R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    // P: [570.3422241210938, 0.0, 319.5,
    // 0.0, 0.0, 570.3422241210938,
    // 239.5, 0.0, 0.0,
    // 0.0, 1.0, 0.0]
    fx_ = 570.3422241210938;
    fy_ = 570.3422241210938;
    px_ = 319.5;
    py_ = 239.5;
    // Initialize the april tag detector.
    tag_detector_ = new AprilTags::TagDetector(tag_code_);
  }

  void SetAprilTagCodeFamily(AprilTags::TagCodes tag_code) {
    tag_code_ = tag_code;
    // Reinitialize the tag detector.
    tag_detector_ = new AprilTags::TagDetector(tag_code_);
  }

  void SetAprilTagCodeSize(double tag_size) {
    tag_size_ = tag_size;
  }

  Eigen::Isometry3d GetRecentAprilTagPose() const {
    return april_tag_pose_;
  }

  bool CheckDetectionInLastImage() const {
    return num_detections_last_image > 0;
  }

  void Run() {
    // Initialize the camera interface.
    camera_interface_ = new pcl::io::OpenNI2Grabber();
    boost::function<void (const boost::shared_ptr<pcl::io::Image>&,
      const boost::shared_ptr<pcl::io::DepthImage>&, float)> image_fun_ =
      boost::bind (&AprilTagPerception::image_cb1, this, _1, _2, _3);
    camera_interface_->registerCallback(image_fun_);
    camera_interface_->start ();
  }

  void Stop() {
    camera_interface_->stop();
  }

 private:
  void image_cb1(const boost::shared_ptr<pcl::io::Image>& rgb_image,
    const boost::shared_ptr<pcl::io::DepthImage>& depth_image,
    float reciprocalFocalLength) {
      bool m_timing = true;
      cv::Mat cv_image = cv::Mat(rgb_image->getHeight(), rgb_image->getWidth(),
        CV_8UC3);
      rgb_image->fillRGB(cv_image.cols, cv_image.rows, cv_image.data,
        cv_image.step);
      // Opencv uses bgr convention.
      cv::cvtColor(cv_image, cv_image, CV_RGB2BGR);
      // Depth image to opencv.
      // cv::Mat D1 = cv::Mat(int(depth_image->getHeight()), int(depth_image->getWidth()), CV_32F);
      // depth_image->fillDepthImage(D1.cols, D1.rows,(float *)D1.data,D1.step);
      cv::Mat image_gray;
      cv::cvtColor(cv_image, image_gray, CV_BGR2GRAY);
      vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(
        image_gray);

      // print out each detection
      cout << detections.size() << " tags detected:" << endl;
      for (int i=0; i<detections.size(); i++) {
        extract_detection(detections[i]);
      }
      num_detections_last_image = detections.size();
      // // show the current image including any detections
      // if (true) {
      //   for (int i=0; i<detections.size(); i++) {
      //     // also highlight in the image
      //     detections[i].draw(image_gray);
      //   }
      //   imshow("apriltag_detection", image_gray); // OpenCV call
      // }
      // cv::waitKey(0);
    }

  void extract_detection(AprilTags::TagDetection& detection) {
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(tag_size_, fx_, fy_, px_, py_,
      translation, rotation);

    Eigen::Matrix3d F;
    F << 1, 0,  0,
         0,  -1,  0,
         0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    april_tag_pose_.linear() = fixed_rot;
    april_tag_pose_.translation() = translation;
    std::cout << april_tag_pose_.linear() << std::endl;
    std::cout << april_tag_pose_.translation() << std::endl;
  }

  pcl::Grabber* camera_interface_;
  // April Detector.
  AprilTags::TagDetector* tag_detector_;
  // Property of the april tag.
  AprilTags::TagCodes tag_code_;
  double tag_size_;
  // Instrinsics of the rgb camera. Focal Length and optical center.
  double fx_;
  double fy_;
  double px_;
  double py_;
  // The current perceived april tag pose w.r.t. camera frame.
  Eigen::Isometry3d april_tag_pose_;
  // The number of detection in the the last seen image.
  int num_detections_last_image;
};


class IiwaState {
 public:
  IiwaState(const RigidBodyTree<double>& iiwa)
      : iiwa_(iiwa),
        cache_(iiwa.CreateKinematicsCache()),
        q_(VectorX<double>::Zero(iiwa.get_num_positions())),
        v_(VectorX<double>::Zero(iiwa.get_num_velocities())),
        ext_trq_(VectorX<double>::Zero(iiwa.get_num_actuators())) {
    DRAKE_DEMAND(iiwa.get_num_positions() == iiwa.get_num_velocities());
    DRAKE_DEMAND(iiwa.get_num_actuators() == iiwa.get_num_velocities());
  }

  void UpdateState(const lcmt_iiwa_status& msg) {
    // Check msg.
    DRAKE_DEMAND(msg.num_joints == iiwa_.get_num_positions());

    const double cur_time = msg.utime / 1e6;

    // Do velocity update first.
    if (init_) {
      // TODO need to filter.
      const double dt = cur_time - time_;
      for (int i = 0; i < msg.num_joints; ++i) {
        v_[i] = (msg.joint_position_measured[i] - q_[i]) / dt;
      }
    } else {
      v_.setZero();
    }

    // Update time, position, and torque.
    time_ = cur_time;
    for (int i = 0; i < msg.num_joints; ++i) {
      q_[i] = msg.joint_position_measured[i];
      ext_trq_[i] = msg.joint_torque_external[i];
    }

    // Update kinematics.
    cache_.initialize(q_, v_);
    iiwa_.doKinematics(cache_);

    init_ = true;
  }

  const KinematicsCache<double>& get_cache() const { return cache_; }
  const VectorX<double>& get_q() const { return q_; }
  const VectorX<double>& get_v() const { return v_; }
  const VectorX<double>& get_ext_trq() const { return ext_trq_; }
  double get_time() const { return time_; }

 private:
  const RigidBodyTree<double>& iiwa_;
  KinematicsCache<double> cache_;
  double time_{kUninitTime};

  VectorX<double> q_;
  VectorX<double> v_;
  VectorX<double> ext_trq_;

  bool init_{false};
};

class RobotPlanRunner {
 public:
  RobotPlanRunner(const RigidBodyTree<double>& robot,
                  const std::string& end_effector_name,
                  const VectorX<double>& q_ini)
      : robot_(robot),
        end_effector_(*robot_.FindBody(end_effector_name)),
        state_(robot_),
        q_ini_(q_ini) {
    lcm_.subscribe(kLcmStatusChannel, &RobotPlanRunner::HandleStatus, this);
  }

  PiecewisePolynomial<double> SplineToDesiredConfiguration(
      const VectorX<double>& q_d, double duration) const {
    std::vector<double> times = {state_.get_time(),
                                 state_.get_time() + duration};
    std::vector<MatrixX<double>> knots = {state_.get_q(), q_d};
    MatrixX<double> zero =
        MatrixX<double>::Zero(robot_.get_num_velocities(), 1);
    return PiecewisePolynomial<double>::Cubic(times, knots, zero, zero);
  }

  // Make a 3 x 3 grid of end effector pose.
  std::vector<Isometry3<double>> GenerateGridOfCalibrateEndEffectorPose(
      double dy, double dz, double rot_z, double rot_y) const {
    std::vector<Isometry3<double>> ret;

    // The rotation from the end effector frame to the last link frame =
    // [0, 0, -1;
    //  0, 1, 0;
    //  1, 0, 0];

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        Isometry3<double> pose = Isometry3<double>::Identity();
        pose.translation()[1] = (1 - j) * dy;
        pose.translation()[2] = (-1 + i) * dz;

        pose.linear() =
            (AngleAxis<double>((-1 + j) * rot_z, Vector3<double>::UnitZ()) *
             AngleAxis<double>((-1 + i) * rot_y, Vector3<double>::UnitY()))
                .toRotationMatrix();
        ret.push_back(pose);
      }
    }

    return ret;
  }

  std::vector<VectorX<double>> GenerateCalibrateConfigurations(
      const VectorX<double>& q_center) {
    std::vector<VectorX<double>> ret;

    KinematicsCache<double> cache = robot_.CreateKinematicsCache();
    // Do fk(q).
    cache.initialize(q_center);
    robot_.doKinematics(cache);
    Isometry3<double> X_WE_center =
        robot_.CalcBodyPoseInWorldFrame(cache, end_effector_);

    std::vector<Isometry3<double>> poses =
        GenerateGridOfCalibrateEndEffectorPose(0.1, 0.1, 0.2, 0.2);

    const Vector3<double> pos_tol(0.001, 0.001, 0.001);
    const double rot_tol = 0.01;

    // This is a hack.
    RigidBodyTree<double>* cao_robot = (RigidBodyTree<double>*)(&robot_);

    // For each pose, solve an ik, and save the q.
    for (const auto& pose : poses) {
      // End effector pose in the the world.
      Isometry3<double> X_WE = X_WE_center * pose;

      //////////////////////////////////////////////////////////
      std::vector<RigidBodyConstraint*> constraint_array;
      IKoptions ikoptions(cao_robot);

      Vector3<double> pos_lb = X_WE.translation() - pos_tol;
      Vector3<double> pos_ub = X_WE.translation() + pos_tol;

      WorldPositionConstraint pos_con(cao_robot, end_effector_.get_body_index(),
                                      Vector3<double>::Zero(), pos_lb, pos_ub,
                                      Vector2<double>::Zero());

      constraint_array.push_back(&pos_con);

      // Adds a rotation constraint.
      WorldQuatConstraint quat_con(cao_robot, end_effector_.get_body_index(),
                                   math::rotmat2quat(X_WE.linear()), rot_tol,
                                   Vector2<double>::Zero());
      constraint_array.push_back(&quat_con);

      VectorX<double> q_res = q_center;
      int info;
      std::vector<std::string> infeasible_constraints;
      inverseKin(cao_robot, q_center, q_center, constraint_array.size(),
                 constraint_array.data(), ikoptions, &q_res, &info,
                 &infeasible_constraints);
      //////////////////////////////////////////////////////////

      ret.push_back(q_res);

      std::cout << q_res.transpose() << "\n";
    }

    return ret;
  }

  void Run() {
    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = robot_.get_num_positions();
    iiwa_command.joint_position.resize(robot_.get_num_positions(), 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(robot_.get_num_positions(), 0.);

    double cmd_time = kUninitTime;

    constexpr int GOTO = 0;
    constexpr int WAIT = 1;
    constexpr int NOOP = 2;

    int STATE = GOTO;
    bool state_init = true;
    double state_t0;

    // traj for GOTO state.
    PiecewisePolynomial<double> traj;

    const double kTrajTime = 2;
    const double kWaitTime = 2;

    std::vector<VectorX<double>> q_calib =
        GenerateCalibrateConfigurations(q_ini_);
    size_t calib_index = 0;

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || state_.get_time() == cmd_time) {
      }

      switch (STATE) {
        case GOTO: {
          if (state_init) {
            traj = SplineToDesiredConfiguration(q_calib[calib_index], kTrajTime);
            calib_index++;

            state_init = false;
            state_t0 = state_.get_time();
          }

          q_d_ = traj.value(state_.get_time());

          if (state_.get_time() - state_t0 > kTrajTime) {
            STATE = WAIT;
            state_init = true;
          }

          break;
        }

        case WAIT: {
          if (state_init) {
            state_init = false;
            state_t0 = state_.get_time();
          }

          if (state_.get_time() - state_t0 > kWaitTime) {
            if (calib_index >= q_calib.size()) {
              STATE = NOOP;
            } else {
              STATE = GOTO;
            }
            state_init = true;
          }
          break;
        }

        case NOOP: {
          break;
        }
      }

      // Make msg.
      iiwa_command.utime = static_cast<int64_t>(state_.get_time() * 1e6);
      for (int i = 0; i < robot_.get_num_positions(); i++) {
        iiwa_command.joint_position[i] = q_d_[i];
      }

      // Send command
      lcm_.publish(kLcmCommandChannel, &iiwa_command);
      // Log time of command.
      cmd_time = state_.get_time();
    }
  }

 private:
  void HandleStatus(const lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    // iiwa_status_ = *status;
    state_.UpdateState(*status);
  }

  lcm::LCM lcm_;
  const RigidBodyTree<double>& robot_;
  const RigidBody<double>& end_effector_;
  // lcmt_iiwa_status iiwa_status_;
  IiwaState state_;
  VectorX<double> q_d_;

  VectorX<double> q_ini_;
};

int do_main() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(kPath,
        multibody::joints::kFixed, nullptr,
        tree.get());

  // Joint angles for the "center" pose, jjz need to update this.
  VectorX<double> q_center = VectorX<double>::Zero(7);
  q_center[1] = 45. / 180. * M_PI;
  q_center[3] = -90. / 180. * M_PI;
  q_center[5] = -45. / 180. * M_PI;

  RobotPlanRunner runner(*tree, kEEName, q_center);
  AprilTagPerception test;
  test.Run();
//  boost::this_thread::sleep (boost::posix_time::seconds (5));
  runner.Run();
  test.Stop();
  return 0;
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::kuka_iiwa_arm::do_main(); }
