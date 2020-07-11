/**
 * @file   InsUtils.h
 * @brief  Tools required to process inertial data
 * @author Cagri, Ryan
 */

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>

#include <core_navigation/InsConst.h>

namespace INS {

typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;

typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, 6, 1> Vector6;
typedef Eigen::Matrix<double, 3, 3> Matrix3;

Vector3 calc_gravity(const double latitude, const double height);

Matrix3 skew_symm(const Vector3 vec);

Matrix3 eul_to_dcm(double phi, double theta, double psi);

Matrix insErrorStateModel_LNF(double R_EPlus, double R_N, Vector3 insLLHPlus, Vector3 insVelPlus, double dt, Matrix3 CbnPlus, double omega_ie,Vector3 omega_n_in,Vector3 f_ib_b,double gravity);

Matrix calc_Q(double R_N, double R_E, Vector3 insLLHPlus, double dt, Matrix3 CbnPlus,Vector3 f_ib_b);

}
