#include <chrono>
#include <memory>
#include <cmath>
#include <functional>
#include <thread>
#include <future>

#include <Eigen/Dense>

#include "arm_attributes.h"

//Forward kinematics: joint angles to cartesian
std::vector<Eigen::Vector3d> fk(double t1, double t2, double t3, double t4, double t5, double t6) {

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

	//roll about X
	Eigen::MatrixXd rot4(3, 3);
	rot4 << 1, 0, 0,
		0, cos(t4), -sin(t4),
		0, sin(t4), cos(t4);

	//roll about Y
	Eigen::MatrixXd  rot6(3, 3);
	rot6 << cos(t6), -sin(t6), 0,
		sin(t6), cos(t6), 0,
		0, 0, 1;

	//pitch about Z
	Eigen::MatrixXd rot5(3, 3);
	rot5 << cos(t5), 0, sin(t5),
		0, 1, 0,
		-sin(t5), 0, cos(t5);

	//defining joint positons after transformations
	Eigen::Vector3d link1_end = rot1 * link1;
	Eigen::Vector3d link2_end = rot1 * rot2 * link2 + link1_end;
	Eigen::Vector3d link3_end = rot1 * rot2 * rot3 * link3 + link2_end;
	Eigen::Vector3d link4_end = rot4 * link4 + link3_end;
	Eigen::Vector3d link5_end = rot4 * rot5 * link5 + link4_end;
	Eigen::Vector3d link6_end = rot4 * rot5 * rot6 * link6 + link5_end;

	std::vector<Eigen::Vector3d> joint_positions_eigen { link1_end, link2_end, link3_end, link4_end, link5_end,link6_end};

	return joint_positions_eigen;
}
