//#include "perception.h"
#include <gpg/candidates_generator.h>
#include <gpg/hand_search.h>
#include <gpg/config_file.h>
#include <vector>
#include <string>
#include <memory>

struct AntiPodalGrasp {
	Eigen::Isometry3d hand_pose;
	double score;
};
// This class implements anti-podal grasp geneneration. Mostly a wrapper over
// Andreas Ten Pan's gpg package with slight internal modification.

class AntiPodalGraspPlanner {
 public:
	AntiPodalGraspPlanner(std::string config_file);
	void SetInputCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> cloud);
	std::vector<AntiPodalGrasp> GenerateAntipodalGrasp();
 private:
 	std::unique_ptr<CandidatesGenerator> candidates_generator;
 	std::unique_ptr<CloudCamera> cloud_camera;	
};