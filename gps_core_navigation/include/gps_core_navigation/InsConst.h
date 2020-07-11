/**
 * @file   InsConst.h
 * @brief  House values that are repeatedly used
 * @author Cagri
 */

#pragma once

namespace INS {

const double omega_ie = 7.292115e-5;   // rotation of Earth in rad/sec
const double Ro = 6378137.0;   //WGS84 Equatorial Radius
const double Rp = 6356752.31425; //WGS84 Polar Radius
const double flat= 1.0/298.257223563; // WGS84 Earth flattening
const double ecc = 0.0818191909425; // WGS84 Eccentricity 0.0818191908426
const double t_const = pow((1.0 - flat),2.0); // constant
const double wheel_radius=0.11; //meters TODO ~0.12
const double wheelbase=0.5; //meters TODO
const double gravity= 9.80665;
const double PI = 3.14159265358979;

}
