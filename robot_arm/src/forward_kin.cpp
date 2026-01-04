#include <chrono>
#include <memory>
#include <cmath>
#include <functional>
#include <thread>
#include <future>

#include <Eigen/Dense>

#include "arm_attributes.h"

//Forward kinematics: joint angles to cartesian
std::vector<Eigen::Vector3d> fk(double t1, double t2, double t3) {

	//Rotation transformation matrices
	// Rotation about Z
	Eigen::MatrixXd rot1(3, 3);
	rot1 << cos(t1), -sin(t1), 0,
		sin(t1), cos(t1), 0,
		0, 0, 1;

	//Y
	Eigen::MatrixXd rot2(3, 3);
	rot2 << cos(t2), 0, sin(t2),
		0, 1, 0,
		-sin(t2), 0, cos(t2);
	//Y
	Eigen::MatrixXd rot3(3, 3);
	rot3 << cos(t3), 0, sin(t3),
		0, 1, 0,
		-sin(t3), 0, cos(t3);

	Eigen::Vector3d link3_eff(0, l3+l4+l5, 0);

	//defining joint positons after transformations
	Eigen::Vector3d link1_end = rot1 * link1;
	Eigen::Vector3d link2_end = rot1 * rot2 * link2 + link1_end;
	Eigen::Vector3d link3_end = rot1 * rot2 * rot3 * link3_eff + link2_end;
	

	std::vector<Eigen::Vector3d> joint_positions_eigen { link1_end, link2_end, link3_end};

	return joint_positions_eigen;
}
