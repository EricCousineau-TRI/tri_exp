#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <iostream>

// This class provides command line control of robot for rapid prototyping 
// of robot experiments.

// Global variables.
const std::string kPath =
    "/home/user/drake_siyuan/drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

drake::jjz::JjzController robot_controller;
RigidBodyFrame<double> tool_frame;
RigidBodyTree<double> tree;

// "SetTool". Set tool transform with respect to link 7. This could be the 
// gripper frame or the camera frame.
void SetToolTransform(Eigen::Isometry3d tf);

// "GotoQ". Linear movement in joint space.
void MoveToJointPosition(const Eigen::VectorXd & q, double duration);

// "GetQ". Get joint position and print on terminal.
Eigen::VectorXd GetJointPosition();

// "GotoC". Linear movement in Cartesian space.
void MoveToCartesianPosition();

// "GetC". Get cartesian pose and print on terminal.
void GetCartesianPosition();

// "GuardedMove". Move in a certain direction until a 
// certain amount of force is sensed.
void MoveUntilTouch();

// "Close". Close the gripper.
void CloseGripper();

// "Open". Open the gripper.
void OpenGripper();

void Run() {

}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    //split(s, delim, std::back_inserter(elems));
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

int main() {
	std::vector<std::string> x = split("gq 1 0 100 20 -30 40 75", ' ');
	for (int i = 0; i < x.size(); ++i) {
		std::cout << x[i] << std::endl;
	}
	return 0;
}