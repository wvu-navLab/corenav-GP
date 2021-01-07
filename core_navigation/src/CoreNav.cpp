#include <core_navigation/CoreNav.h>
#include <core_navigation/InsConst.h>
#include <core_navigation/InsUtils.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <geometry_msgs/TransformStamped.h>
#include <hw_interface_plugin_roboteq/RoboteqData.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <core_navigation/ICEclass.h>
//#include <EigenRand/EigenRand>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

CoreNav::CoreNav() : initialized_(false){
}

CoreNav::~CoreNav(){
}
// FILTER STEPS
void CoreNav::Propagate(){

        // ROS_WARN("Inside of the Propagate function\n");
        dt_imu_ = imu_stamp_curr_ - imu_stamp_prev_; //OK
        count++;
        omega_b_ib_ << imu[3] - bg_(0), imu[4] - bg_(1), imu[5] - bg_(2); // not the same
        f_ib_b_ << imu[0] - ba_(0), imu[1] - ba_(1), imu[2] - ba_(2);
        psiEst_minus=ins_attMinus_[2];
        //Attitude Update
        CoreNav::Matrix3 CnbMinus = CoreNav::eul_to_dcm(ins_attMinus_[0],ins_attMinus_[1],ins_attMinus_[2]);

        CbnMinus=CnbMinus.transpose();
        omega_n_ie_ << INS::omega_ie*cos(ins_posMinus_[0]), 0.0, (-1.0)*INS::omega_ie*sin(ins_posMinus_[0]);
        Omega_b_ib_ <<  0, -omega_b_ib_(2), omega_b_ib_(1), omega_b_ib_(2), 0, -omega_b_ib_(0),-omega_b_ib_(1), omega_b_ib_(0), 0;
        Omega_n_ie_ <<  0, INS::omega_ie*sin(ins_posMinus_(0)), 0, -INS::omega_ie*sin(ins_posMinus_(0)), 0,-INS::omega_ie*cos(ins_posMinus_(0)),0, INS::omega_ie*cos(ins_posMinus_(0)), 0;
        // Radius of Curvature for North-South Motion (eq. 2.105)
        double R_N = INS::Ro*(1.0-pow(INS::ecc,2.0))/pow((1.0-pow(INS::ecc,2.0)*pow(sin(ins_posMinus_(0)),2.0)),(3.0/2.0));
        // Radius of Curvature in East-West Direction (eq. 2.106)
        double R_E = INS::Ro/pow((1.0-pow(INS::ecc,2.0)*pow(sin(ins_posMinus_(0)),2.0)),(1.0/2.0));
        //rotation rate vector
        omega_n_en_ << ins_velMinus_(1)/(R_E+ins_posMinus_(2)),
        -ins_velMinus_(0)/(R_N+ins_posMinus_(2)),
        (-ins_velMinus_(1)*tan(ins_posMinus_(0)))/(R_E+ins_posMinus_(2));
        Omega_n_en_ << 0, -omega_n_en_(2), omega_n_en_(1), omega_n_en_(2), 0, -omega_n_en_(0), -omega_n_en_(1), omega_n_en_(0), 0;
        omega_n_in_ = omega_n_en_ + omega_n_ie_;
        //integrate considering body-rate, Earth-rate, and craft-rate
        CoreNav::Matrix3 eye3=Eigen::Matrix3d::Identity();
        CoreNav::Matrix3 zeros3=Eigen::Matrix3d::Zero(3,3);
        CoreNav::Matrix3 CbnPlus;
        CbnPlus=CbnMinus*(eye3+Omega_b_ib_*dt_imu_)-(Omega_n_ie_+ Omega_n_en_)*CbnMinus*dt_imu_;

        ins_att_= CoreNav::dcm_to_eul( CbnPlus );

        CoreNav::Vector3 V_n_ib;
        //specific-force transformation
        V_n_ib=(1.0/2.0)*(CbnMinus+CbnPlus)*f_ib_b_*dt_imu_;
        grav_ = CoreNav::calc_gravity(ins_posMinus_(0),ins_posMinus_(2));
        ins_vel_= ins_velMinus_ + V_n_ib +  (grav_ - (Omega_n_en_+2.0*(Omega_n_ie_))*ins_velMinus_)*dt_imu_;

        // Position Update
        double heightPlus=ins_posMinus_(2)-(dt_imu_/2.0)*(ins_velMinus_(2)+ins_vel_(2)); //ins_pos_(2)
        double latPlus = ins_posMinus_(0)+(dt_imu_/2.0)*(ins_velMinus_(0)/(R_N+ins_posMinus_(2))+ins_vel_(0)/(R_N+heightPlus)); //ins_pos_(0)
        double R_EPlus=INS::Ro/pow((1.0-pow(INS::ecc,2.0)*pow(sin(latPlus),2.0)),(1.0/2.0));
        double lonPlus=ins_posMinus_(1)+(dt_imu_/2.0)*(ins_velMinus_(1)/((R_E+ins_posMinus_(2))*cos(ins_posMinus_(0)))+ins_vel_(1)/((R_EPlus+heightPlus)*cos(latPlus)));   //ins_pos_(1)
        ins_pos_ << latPlus,lonPlus,heightPlus;

        // State Transition Matrix
        STM_ = CoreNav::insErrorStateModel_LNF(R_EPlus, R_N, ins_pos_, ins_vel_, dt_imu_, CbnPlus, INS::omega_ie,omega_n_in_, f_ib_b_, INS::gravity);

        // Error Propagation
        error_states_ = STM_*error_states_;

        // Q Matrix
        Q_ = CoreNav::calc_Q(R_N,R_E,ins_pos_, dt_imu_, CbnPlus, f_ib_b_);

        // P Matrix
        P_=STM_*P_*STM_.transpose()+ Q_;


        // Integrate IMU measurements for odometry Update
        CoreNav::Matrix3 Cn2bPlus=CbnPlus.transpose();
        CoreNav::Vector3 omega_b_ie=Cn2bPlus*omega_n_ie_;
        CoreNav::Vector3 omega_b_ei=-1.0*omega_b_ie;
        CoreNav::Vector3 omega_b_eb=omega_b_ei+omega_b_ib_;
        CoreNav::Matrix3 ins_vel_ss; //ins_vel_ skew symmetric
        ins_vel_ss = CoreNav::skew_symm(ins_vel_);
        CoreNav::Vector3 Val_H21(0,cos(ins_att_(0)),sin(ins_att_(0)));

        H11_ = H11_ + (eye3.row(0)*Cn2bPlus*ins_vel_ss*dt_imu_).transpose();
        H12_ = H12_ + (eye3.row(0)*Cn2bPlus*dt_imu_).transpose();
        H21_ = H21_ + (sin(ins_att_(1))*Val_H21.transpose()*Cn2bPlus*dt_imu_).transpose();
        H31_ = H31_ + (eye3.row(1)*Cn2bPlus*ins_vel_ss*dt_imu_).transpose();
        H32_ = H32_ + (eye3.row(1)*Cn2bPlus*dt_imu_).transpose();
        H41_ = H41_ + (eye3.row(2)*Cn2bPlus*ins_vel_ss*dt_imu_).transpose();
        H42_ = H42_ + (eye3.row(2)*Cn2bPlus*dt_imu_).transpose();

        // Measurement Innovation -- integration part for INS -- eq 16.42
        CoreNav::Vector3 lb2f(-T_r_/2, 0.0, -INS::wheel_radius*2);
        double tmp1 = eye3.row(0)*(Cn2bPlus*ins_vel_ + (CoreNav::skew_symm(omega_b_eb)*lb2f));
        double tmp2 = eye3.row(1)*(Cn2bPlus*ins_vel_ + (CoreNav::skew_symm(omega_b_eb)*lb2f));
        double tmp3 = eye3.row(2)*(Cn2bPlus*ins_vel_ + (CoreNav::skew_symm(omega_b_eb)*lb2f));

        z11_ =z11_ + tmp1*dt_imu_;
        z21_ =z21_ + cos(ins_att_(1))*dt_imu_;
        z31_ =z31_ + tmp2* dt_imu_;
        z41_ =z41_ + tmp3* dt_imu_;

        //NonHolonomic Update
        CoreNav::NonHolonomic(ins_vel_, ins_att_, ins_pos_, error_states_, P_, omega_b_ib_);

        // Zero-Updates
        if ( std::abs(rearVel_) < 0.005) //
        {
              CoreNav::zeroUpdate(ins_vel_, ins_att_, ins_pos_, error_states_, P_, omega_b_ib_);
        }

        ins_attMinus_=ins_att_;
        ins_velMinus_=ins_vel_;
        ins_posMinus_=ins_pos_;


        ba_(0)=ba_(0)+error_states_(9);
        ba_(1)=ba_(1)+error_states_(10);
        ba_(2)=ba_(2)+error_states_(11);
        bg_(0)=bg_(0)+error_states_(12);
        bg_(1)=bg_(1)+error_states_(13);
        bg_(2)=bg_(2)+error_states_(14);

        error_states_.segment(9,6)<<Eigen::VectorXd::Zero(6);

        ins_enu_ << CoreNav::llh_to_enu(ins_pos_[0],ins_pos_[1],ins_pos_[2]);
        ins_cn_<<ins_att_,ins_vel_,ins_enu_;
        ins_xyz_ << CoreNav::llh_to_xyz(ins_pos_[0],ins_pos_[1],ins_pos_[2]);
        ins_pos_llh_ << ins_pos_[0]*180.0/INS::PI,ins_pos_[1]*180.0/INS::PI, ins_pos_[2];

        CoreNav::Vector6 llhstate;
        llhstate << ins_pos_(0),ins_pos_(1),ins_pos_(2),error_states_(6),error_states_(7),error_states_(8);

        PublishStates(ins_att_, attitude_pub_);
        PublishStates(ins_vel_, velocity_pub_);
        PublishStates(ins_pos_, position_pub_);

        // Eigen::MatrixXd covar(6,6);
        // covar = P_.block(3,3,6,6);
        // cout << covar << endl;
        // cout <<"***---------***"<< endl;
        PublishStateswCov(llhstate, enu_pub_);
        PublishStatesenu(ins_cn_,enu_pub2_);
        PublishStates(ins_xyz_, ins_xyz_pub_);
        PublishStates(ins_pos_llh_, pos_llh_pub_);

        PublishStatesCN(ins_cn_, cn_pub_);


        return;
}
void CoreNav::Update(){

        odomUptCount=odomUptCount+1;
        // ROS_INFO_STREAM("odomUptCount: " << odomUptCount);
        velFrontLeft_ = -joint[0]*INS::wheel_radius;
        velFrontRight_=  joint[1]*INS::wheel_radius;
        velBackLeft_  = -joint[2]*INS::wheel_radius;
        velBackRight_ =  joint[3]*INS::wheel_radius;

        rearVel_=(velBackLeft_+velBackRight_)/2.0;

        double frontVel =(velFrontLeft_+velFrontRight_)/2.0;
        headRate_=(velBackLeft_-velBackRight_)/T_r_;
        dt_odo_ = joint_stamp_curr_ - joint_stamp_prev_;

        CoreNav::Matrix3 Cn2bUnc=CoreNav::eul_to_dcm(ins_att_(0),ins_att_(1),ins_att_(2));
        H11_ =-H11_/dt_odo_;
        H12_ =-H12_/dt_odo_;
        H21_ = H21_*(ins_att_(2)-psiEst_minus)/(dt_odo_*dt_odo_);
        H31_ =-H31_/dt_odo_;
        H32_ =-H32_/dt_odo_;
        H24_ =-(cos(ins_att_(1))*eye3.row(2)*Cn2bUnc.transpose())/dt_odo_;
        H41_ =-H41_/dt_odo_;
        H42_ =-H42_/dt_odo_;

        double z1_odom=rearVel_*(1-s_or_);
        double z2_odom=headRate_*(1-s_or_)-((z11_/dt_odo_)/T_r_)*s_delta_or_;
        double z1_ins=z11_;
        double z2_ins=(ins_att_(2)-psiEst)*z21_;// psiEst in here should be the heading between the updates (i.e., psiEst(1), psiEst(26), psiEst(51)

        if (abs(z2_ins) > 0.5) {
                z2_ins=0.0;
        }

        double z3_ins=z31_;
        double z4_ins=z41_;
        z11_=z1_odom-z1_ins/dt_odo_;
        z21_=z2_odom-z2_ins/pow(dt_odo_,2.0);
        z31_=0.0-z3_ins/dt_odo_;
        z41_=0.0-z4_ins/dt_odo_;
        Z_<<z11_,z21_,z31_,z41_;

        H_.row(0)<<H11_.transpose(), H12_.transpose(), zeros3.row(0), zeros3.row(0),zeros3.row(0);
        H_.row(1)<<H21_.transpose(), zeros3.row(0), zeros3.row(0), H24_.transpose(),zeros3.row(0);
        H_.row(2)<<H31_.transpose(), H32_.transpose(), zeros3.row(0), zeros3.row(0),zeros3.row(0);
        H_.row(3)<<H41_.transpose(), H42_.transpose(), zeros3.row(0), zeros3.row(0),zeros3.row(0);

        Eigen::RowVectorXd res(4);

        res = Z_-H_*error_states_ ;

        //  ADD UPDATE FUNCTION HERE  // *********************************************************
      //****************************************************************************************

        //UpdateR(res);  // updating R with ICE (Ryan)
        //res = res-ice_mean;
        //huberUpdate();  // Huber filter (Karlgraad)
        //normalUpdate(res); // Standard filter (Cagri)
        //adaptiveUpdate(res);
        //orkf1Update(res);  // iid noise Variational Filter by Agememnoni An outlier robust Kalman Filter
        //orkf1DriftUpdate(res);  // drifting noise Variational Filter by Agememnoni An outlier robust Kalman Filter
        //orkf2Update(res);  // Variational Filter by Sarkka Recursive outlier-robust filtering and smoothing for nonlinear systems using the multivariate Student-t distribution
        orkf3Update(res);  // A Novel Adaptive Kalman Filter with Inaccurate Process and Measurement Noise Covariance Matrices by Yulong Huang
        //orkf4Update(res);  // Scaling filter by G Chang  Robust Kalman filtering based on Mahalanobis distance as outlier judging criterion
        //orkf5Update(res);  // Adaptive filter by Akhlaghi  https://arxiv.org/pdf/1702.00884.pdf
        //orkf6Update(res);  // Covariance Scaling filter by Yang Adaptively robust filtering for kinematic geodetic positioning


        ins_att_ = CoreNav::dcm_to_eul((Eigen::MatrixXd::Identity(3,3)- CoreNav::skew_symm(error_states_.segment(0,3)))*Cn2bUnc.transpose());
        ins_pos_ = ins_pos_ - error_states_.segment(6,3);
        ins_vel_ = ins_vel_ - error_states_.segment(3,3);

        ins_attMinus_=ins_att_;
        ins_velMinus_=ins_vel_;
        ins_posMinus_=ins_pos_;

        psiEst=ins_att_[2];

        error_states_.segment(0,9)<<Eigen::VectorXd::Zero(9);


        double vlin=eye3.row(0)*(Cn2bUnc*ins_vel_);
        double slip= std::max(std::max(((velFrontRight_)-vlin)/(velFrontRight_),((velBackRight_)-vlin)/(velBackRight_)),std::max(((velFrontLeft_)-vlin)/(velFrontLeft_),((velBackLeft_)-vlin)/(velBackLeft_)));


        if (std::abs(rearVel_)<0.001)
        {
                slip=0.0;
        }
        if (slip<-1.0)
        {
                slip=-1.0;
        }
        if (slip>1.0)
        {
                slip=1.0;
        }

        if (std::isnan(vlin)) {
          ROS_ERROR_ONCE("VELOCITY IS NaN - Restart required");
        }

        H11_=zeros3.row(0);
        H12_=zeros3.row(0);
        H21_=zeros3.row(0);
        H31_=zeros3.row(0);
        H32_=zeros3.row(0);
        H24_=zeros3.row(0);
        H41_=zeros3.row(0);
        H42_=zeros3.row(0);

        z11_=0.0;
        z21_=0.0;
        z31_=0.0;
        z41_=0.0;

        slip_cn_<<slip, rearVel_,vlin;
        PublishStatesSlip(slip_cn_, slip_pub_);
        // ROS_INFO_STREAM("insattitudeafter " << ins_att_);
        return;
}
// PSEUDO MEASUREMENT UPDATES
void CoreNav::NonHolonomic(const CoreNav::Vector3 vel, const CoreNav::Vector3 att, const CoreNav::Vector3 llh, CoreNav::Vector15 error_states, Eigen::MatrixXd P, CoreNav::Vector3 omega_b_ib){

  CoreNav::Matrix3 Cnb = CoreNav::eul_to_dcm(att[0],att[1],att[2]);
  CoreNav::Vector3 lf2b(0.0, 0.0, 0.272);

  z_holo.row(0) = -eye3.row(1)*(Cnb*vel-CoreNav::skew_symm(omega_b_ib)*lf2b); //z31
  z_holo.row(1) = -eye3.row(2)*(Cnb*vel-CoreNav::skew_symm(omega_b_ib)*lf2b); //z41
  H_holo.row(0) << zeros3.row(0), -eye3.row(1)*Cnb, zeros3.row(0), zeros3.row(0), zeros3.row(0); //h32
  H_holo.row(1) << zeros3.row(0), -eye3.row(2)*Cnb, zeros3.row(0), zeros3.row(0), zeros3.row(0); //h42

  if (abs(omega_b_ib[2]>0.1)) {
    R_holoS << 0.05;
    z_holoS << -eye3.row(2)*(Cnb*vel-CoreNav::skew_symm(omega_b_ib)*lf2b);
    H_holoS << zeros3.row(0), -eye3.row(2)*Cnb, zeros3.row(0), zeros3.row(0), zeros3.row(0);
    double tempVar1= H_holoS * P * H_holoS.transpose();
    double tempVar2= 1/(tempVar1+0.05);
    Vector15 tempvar3= P * H_holoS.transpose();
    Vector15 tempVar4= tempvar3*tempVar2;
     K_holoS=tempVar4;
     error_states_ = error_states + K_holoS* (z_holoS  - H_holoS * error_states);
     ins_att_ = CoreNav::dcm_to_eul((Eigen::MatrixXd::Identity(3,3)- CoreNav::skew_symm(error_states_.segment(0,3)))*Cnb.transpose());
     ins_vel_ = vel - error_states_.segment(3,3);
     ins_pos_ = llh - error_states_.segment(6,3);
     error_states_.segment(0,9)<<Eigen::VectorXd::Zero(9);
     P_=(Eigen::MatrixXd::Identity(15,15) - K_holoS * H_holoS) * P* ( Eigen::MatrixXd::Identity(15,15) - K_holoS * H_holoS).transpose() + K_holoS * R_holoS * K_holoS.transpose();
  }
  else {
    K_holo = P * H_holo.transpose() * (H_holo * P * H_holo.transpose() + R_holo).inverse();
    error_states_ = error_states + K_holo* (z_holo  - H_holo * error_states);
    ins_att_ = CoreNav::dcm_to_eul((Eigen::MatrixXd::Identity(3,3)- CoreNav::skew_symm(error_states_.segment(0,3)))*Cnb.transpose());
    ins_vel_ = vel - error_states_.segment(3,3);
    ins_pos_ = llh - error_states_.segment(6,3);
    error_states_.segment(0,9)<<Eigen::VectorXd::Zero(9);
    P_=(Eigen::MatrixXd::Identity(15,15) - K_holo * H_holo) * P* ( Eigen::MatrixXd::Identity(15,15) - K_holo * H_holo ).transpose() + K_holo * R_holo * K_holo.transpose();
  }

  return;
}
void CoreNav::zeroUpdate(const CoreNav::Vector3 vel, const CoreNav::Vector3 att, const CoreNav::Vector3 llh, CoreNav::Vector15 error_states, Eigen::MatrixXd P, const CoreNav::Vector3 omega_b_ib){
        countZero++;
        CoreNav::Matrix3 Cnb = CoreNav::eul_to_dcm(att[0],att[1],att[2]);
        CoreNav::Vector3 z_zaru;
        CoreNav::Vector3 z_zupt;
        z_zaru = -omega_b_ib.transpose();
        z_zupt = -vel;
        CoreNav::Vector6 z_zero;
        z_zero.segment(0,3) <<z_zaru;
        z_zero.segment(3,3) <<z_zupt;
        K_zero = P * H_zero.transpose() * (H_zero * P * H_zero.transpose() + R_zero).inverse();
        error_states_ = error_states + K_zero * (z_zero  - (H_zero * error_states));
        ins_att_ = CoreNav::dcm_to_eul((Eigen::MatrixXd::Identity(3,3)- CoreNav::skew_symm(error_states_.segment(0,3)))*Cnb.transpose());
        ins_vel_ = vel - error_states_.segment(3,3);
        ins_pos_ = llh - error_states_.segment(6,3);
        error_states_.segment(0,9)<<Eigen::VectorXd::Zero(9);
        P_=(Eigen::MatrixXd::Identity(15,15) - K_zero * H_zero) * P * ( Eigen::MatrixXd::Identity(15,15) - K_zero * H_zero ).transpose() + K_zero * R_zero * K_zero.transpose();

        return;
}
// STATE TRANSITION and COVARIANCE OF PROCESS NOISE CALCULATION
CoreNav::Matrix CoreNav::insErrorStateModel_LNF(double R_EPlus, double R_N, CoreNav::Vector3 insLLH, CoreNav::Vector3 insVel, double dt, CoreNav::Matrix3 CbnPlus, double omega_ie,CoreNav::Vector3 omega_n_in,CoreNav::Vector3 f_ib_b,double gravity){
        double geoLat= atan2(INS::t_const*sin(insLLH(0)*180.0/INS::PI), cos(insLLH(0)*180.0/INS::PI));
        double rGeoCent  = pow(( pow(INS::Ro,2.0) /( 1.0 + (1.0/(pow(( 1.0 - INS::flat ),2.0)) - 1.0)*pow(sin(geoLat),2.0))),(1.0/2.0));
        double g0 = 9.780318*( 1.0 + (5.3024e-3)*pow(sin(insLLH(0)),2.0) - (5.9e-6)*pow(sin(2*insLLH(0)),2.0) );

        CoreNav::Matrix3 F11;
        F11= (-1)*CoreNav::skew_symm(omega_n_in);

        CoreNav::Matrix3 F12;
        F12.row(0)<<0.0, -1.0/(R_EPlus+insLLH(2)), 0.0; //height
        F12.row(1)<<1.0/(R_N+insLLH(2)), 0.0, 0.0; //height
        F12.row(2)<<0.0, tan(insLLH(0))/(R_EPlus+insLLH(2)), 0.0; //lat/height

        CoreNav::Matrix3 F13;
        F13.row(0)<<omega_ie*sin(insLLH(0)), 0.0, insVel(1)/pow((R_EPlus+insLLH(2)),2.0);
        F13.row(1)<<0.0, 0.0, -insVel(0)/pow((R_N+insLLH(2)),2.0);
        F13.row(2)<<omega_ie * cos(insLLH[0]) + insVel[1] / ((R_EPlus + insLLH[2]) *(cos(insLLH[0])* cos(insLLH[0]))),0.0,-insVel[1] * tan(insLLH[0]) / ((R_EPlus + insLLH[2]) * (R_EPlus + insLLH[2]));
        CoreNav::Matrix3 F21 = (-1.0)*CoreNav::skew_symm(CbnPlus*(f_ib_b));

        CoreNav::Matrix3 F22;
        F22.row(0)<< insVel[2] / (R_N + insLLH[2]), -(2.0 * insVel[1] * tan(insLLH[0]) / (R_EPlus + insLLH[2])) - 2.0 * omega_ie * sin(insLLH[0]),insVel[0] / (R_N + insLLH[2]);
        F22.row(1)<< insVel[1] * tan(insLLH[0]) / (R_EPlus + insLLH[2]) + 2.0 * omega_ie *sin(insLLH[0]),(insVel[0] * tan(insLLH[0]) + insVel[2]) / (R_EPlus + insLLH[2]),insVel[1] / (R_EPlus + insLLH[2]) + 2.0 * omega_ie *cos(insLLH[0]);
        F22.row(2)<< -2.0 * insVel[0] / (R_N + insLLH[2]),-2.0 * (insVel[1] / (R_EPlus + insLLH[2])) - 2.0 * omega_ie * cos(insLLH[0]),0.0;

        CoreNav::Matrix3 F23;
        F23.row(0)<< -(insVel[1] * insVel[1] * ((1.0 / cos(insLLH[0])) * (1.0 / cos(insLLH[0]))) / (R_EPlus + insLLH[2])) - 2.0 *insVel[1] * omega_ie * cos(insLLH[0]), 0.0,insVel[1] * insVel[1] * tan(insLLH[0]) / ((R_EPlus + insLLH[2]) * (R_EPlus + insLLH[2])) - insVel[0] * insVel[2] / ((R_N + insLLH[2]) * (R_N + insLLH[2]));
        F23.row(1)<<  (insVel[0] * insVel[1] * ((1.0 / cos(insLLH[0])) * (1.0 / cos(insLLH[0]))) / (R_EPlus + insLLH[2])) + 2.0 *insVel[0] * omega_ie * cos(insLLH[0]) - 2.0 * insVel[2] * omega_ie * sin(insLLH[0]),0.0, -((insVel[0] * insVel[1] * tan(insLLH[0]) + insLLH[1] * insLLH[2]) / ((R_EPlus + insLLH[2]) * (R_EPlus + insLLH[2])));
        F23.row(2)<< 2.0 * insVel[1] * omega_ie * sin(insLLH[0]), 0.0, (insVel[1] * insVel[1] / ((R_EPlus + insLLH[2]) * (R_EPlus + insLLH[2])) + insVel[0] * insVel[0] / ((R_N + insLLH[2]) *(R_N + insLLH[2]))) - 2.0 * g0 / rGeoCent;

        CoreNav::Matrix3 F32;
        F32.row(0)<<(1.0)/(R_N+insLLH(2)), 0.0, 0.0;
        F32.row(1)<<0.0, (1.0)/((R_EPlus+insLLH(2))*cos(insLLH(0))), 0.0;
        F32.row(2)<<0.0, 0.0, -1.0;

        CoreNav::Matrix3 F33;
        F33.row(0)<<0.0, 0.0, -insVel(0)/pow((R_N+insLLH(2)),2.0);
        F33.row(1)<<(insVel(1)*sin(insLLH(0)))/((R_EPlus+insLLH(2))*pow(cos(insLLH(0)),2.0)), 0.0, -insVel(1)/(pow((R_EPlus+insLLH(2)),2.0)*cos(insLLH(0)));
        F33.row(2)<<0.0, 0.0, 0.0;

        Eigen::Matrix3d PHI11 = Eigen::Matrix3d::Identity(3,3)+F11*dt;
        Eigen::Matrix3d PHI12 = F12*dt;
        Eigen::Matrix3d PHI13 = F13*dt;
        Eigen::Matrix3d PHI15 = CbnPlus*dt;
        Eigen::Matrix3d PHI21 = F21*dt;
        Eigen::Matrix3d PHI22 = Eigen::Matrix3d::Identity()+F22*dt;
        Eigen::Matrix3d PHI23 = F23*dt;
        Eigen::Matrix3d PHI24 = CbnPlus*dt;
        Eigen::Matrix3d PHI32 = F32*dt;
        Eigen::Matrix3d PHI33 = Eigen::Matrix3d::Identity()+F33*dt;

//%Eq:14.72
        CoreNav::Matrix STM(15,15);

        STM<<PHI11, PHI12, PHI13, Eigen::Matrix3d::Zero(3,3),PHI15,
        PHI21, PHI22, PHI23, PHI24, Eigen::Matrix3d::Zero(3,3),
        Eigen::Matrix3d::Zero(3,3), PHI32, PHI33, Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3),
        Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Identity(3,3),Eigen::Matrix3d::Zero(3,3),
        Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Zero(3,3),Eigen::Matrix3d::Identity(3,3);

        return STM;
}
CoreNav::Matrix CoreNav::calc_Q(double R_N, double R_E, CoreNav::Vector3 insLLH, double dt, CoreNav::Matrix3 CbnPlus,CoreNav::Vector3 f_ib_b){
        CoreNav::Matrix3 F21 = (-1.0)*CoreNav::skew_symm(CbnPlus*(f_ib_b));
        Eigen::Matrix3d T_rn_p;
        T_rn_p.row(0)<<1.0/(R_N+insLLH(2)),0.0,0.0;
        T_rn_p.row(1)<<0.0,1.0/((R_E+insLLH(2))*cos(insLLH(0))),0.0;
        T_rn_p.row(2)<<0.0,0.0,-1.0;
        double gg=9.80665;

        double sig_gyro_inRun = 1.6*INS::PI/180/3600; //rad/s
        double sig_ARW = .1*(INS::PI/180)*sqrt(3600)/3600;; //rad

        double sig_accel_inRun = (3.2e-6)*gg; // m/s
        double sig_VRW = 0.008*sqrt(3600)/3600; //m/s

//following 14.2.6 of Groves pp 592
        double Srg= pow(sig_ARW,2)*dt;
        double Sra= pow(sig_VRW,2)*dt;

        double Sbad=pow(sig_accel_inRun,2)/dt;
        double Sbgd=pow(sig_gyro_inRun,2)/dt;

        Eigen::Matrix3d Q11 = (Srg*dt+(1.0/3.0)*Sbgd*pow(dt,3.0))*Eigen::Matrix3d::Identity(3,3);
        Eigen::Matrix3d Q21= ((1.0/2.0)*Srg*pow(dt,2)+(1.0/4.0)*Sbgd*pow(dt,4.0))*F21;
        Eigen::Matrix3d Q12=Q21.transpose();
        Eigen::Matrix3d Q31= ((1.0/3.0)*Srg*pow(dt,3.0)+(1.0/5.0)*Sbgd*pow(dt,5.0))*T_rn_p*F21;
        Eigen::Matrix3d Q13=Q31.transpose();
        Eigen::Matrix3d Q14=Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d Q15=(1.0/2.0)*Sbgd*pow(dt,2.0)*CbnPlus;
        Eigen::Matrix3d Q22= (Sra*dt+(1.0/3.0)*Sbad*pow(dt,3.0))*Eigen::Matrix3d::Identity(3,3)+((1.0/3.0)*Srg*pow(dt,3.0)+(1.0/5.0)*Sbgd*pow(dt,5.0))*(F21*F21.transpose());
        Eigen::Matrix3d Q32= ((1.0/2.0)*Sra*pow(dt,2.0)+(1.0/4.0)*Sbad*pow(dt,4))*T_rn_p+((1.0/4.0)*Srg*pow(dt,4.0)+(1.0/6.0)*Sbgd*pow(dt,6.0))*T_rn_p*(F21*F21.transpose());
        Eigen::Matrix3d Q23=Q32.transpose();
        Eigen::Matrix3d Q24= (1.0/2.0)*Sbad*pow(dt,2.0)*CbnPlus;
        Eigen::Matrix3d Q25= (1.0/3.0)*Sbgd*pow(dt,3.0)*F21*CbnPlus;
        Eigen::Matrix3d Q33= ((1.0/3.0)*Sra*pow(dt,3.0) + (1.0/5.0)*Sbad*pow(dt,5.0))*(T_rn_p*T_rn_p)+((1.0/5.0)*Srg*pow(dt,5.0)+(1.0/7.0)*Sbgd*pow(dt,7.0))*T_rn_p*(F21*F21.transpose())*T_rn_p;
        Eigen::Matrix3d Q34=(1.0/3.0)*Sbad*pow(dt,3.0)*T_rn_p*CbnPlus;
        Eigen::Matrix3d Q35=(1.0/4.0)*Sbgd*pow(dt,4.0)*T_rn_p*F21*CbnPlus;
        Eigen::Matrix3d Q41= Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d Q42= (1.0/2.0)*Sbad*pow(dt,2.0)*(CbnPlus.transpose());
        Eigen::Matrix3d Q43=Q34.transpose();
        Eigen::Matrix3d Q44=Sbad*dt*Eigen::Matrix3d::Identity(3,3);
        Eigen::Matrix3d Q45=Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d Q51= (1.0/2.0)*Sbgd*pow(dt,2.0)*(CbnPlus.transpose());
        Eigen::Matrix3d Q52=(1.0/3.0)*Sbgd*pow(dt,3.0)*F21.transpose()*(CbnPlus.transpose());
        Eigen::Matrix3d Q53=Q35.transpose();
        Eigen::Matrix3d Q54=Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d Q55= Sbgd*dt*Eigen::Matrix3d::Identity(3,3);

        CoreNav::Matrix Q(15,15);
        Q<<Q11,Q12,Q13,Q14,Q15,
        Q21,Q22,Q23,Q24,Q25,
        Q31,Q32,Q33,Q34,Q35,
        Q41,Q42,Q43,Q44,Q45,
        Q51,Q52,Q53,Q54,Q55;
        return Q;
}
// TOOLS
CoreNav::Vector3 CoreNav::calc_gravity(const double latitude, const double height){
        double e2=pow(INS::ecc,2.0);
        double den=1.0-e2*pow(sin(latitude),2.0);
        double Rm=INS::Ro*(1.0-e2)/pow(den,(3.0/2.0));
        double Rp=INS::Ro/pow(den,(1.0/2.0));
        double Top=INS::Ro*pow((pow((1.0-e2),2.0)+pow((sin(latitude)),2.0)+pow((cos(latitude)),2.0)),(1.0/2.0));
        double Bottom=pow((1.0-e2*(pow(sin(latitude),(2.0)))),(1.0/2.0));
        double R_es_e=Top/Bottom;
        double RO=pow(Rp*Rm,(1.0/2.0));
        double g0=9.780318*(1.0+5.3024e-3*pow(sin(latitude),2.0)-5.9e-6*pow(sin(2*latitude),2.0));
        double gravity;

        if(height<0.0)
        {
                gravity=g0*(1.0+height/RO);
        }
        else
        {
                gravity=g0/pow((1.0+height/RO),2.0);
        }
        CoreNav::Vector3 grav(0.0,0.0,gravity);
        return grav;
}
CoreNav::Matrix3 CoreNav::skew_symm(const CoreNav::Vector3 vec){
        CoreNav::Matrix3 ss;
        ss << 0.0, -1.0*vec(2), vec(1),
        vec(2), 0.0, -1.0*vec(0),
        -1.0*vec(1), vec(0), 0.0;

        return ss;
}
CoreNav::Matrix3 CoreNav::eul_to_dcm(double phi, double theta, double psi){
        double cpsi = cos(psi); double spsi = sin(psi);
        double cthe = cos(theta); double sthe = sin(theta);
        double cphi = cos(phi); double sphi = sin(phi);

        CoreNav::Matrix3 c1; //y
        c1.row(0)<<cpsi, spsi, 0.0; //c11
        c1.row(1)<<(-1.0)*spsi, cpsi,0.0; //c12
        c1.row(2)<<0.0, 0.0,1.0;  //c13

        CoreNav::Matrix3 c2; //p
        c2.row(0)<<cthe,0.0,(-1.0)*sthe; //c21
        c2.row(1)<<0.0,1.0,0.0;   //c22
        c2.row(2)<<sthe,0.0,cthe; //c23

        CoreNav::Matrix3 c3; //r
        c3.row(0)<<1.0,0.0,0.0;   //c31
        c3.row(1)<<0.0,cphi,sphi; //c32
        c3.row(2)<<0.0,(-1.0)*sphi,cphi; //c33

        CoreNav::Matrix3 DCMnb= (c3*c2)*c1; //CnbMinus;nav2body
        return DCMnb;
}
CoreNav::Vector3 CoreNav::dcm_to_eul(CoreNav::Matrix3 dcm){
        CoreNav::Vector3 eul;
        eul << std::atan2(dcm(2,1), dcm(2,2)), std::asin(-1*dcm(2,0)), std::atan2(dcm(1,0), dcm(0,0));
        return eul;
}
CoreNav::Vector3 CoreNav::llh_to_enu(double lat, double lon, double height){


        double phi = lat;
        double lambda = lon;
        double h = height;

        double a = 6378137.0000;  //% earth semimajor axis in meters
        double b = 6356752.3142;  //% earth semiminor axis in meters
        double e = sqrt(1-pow((b/a),2));

        double sinphi = sin(phi);
        double cosphi = cos(phi);
        double coslam = cos(lambda);
        double sinlam = sin(lambda);
        double tan2phi = pow((tan(phi)),2);
        double tmp2 = 1 - e*e;
        double tmpden = sqrt( 1 + tmp2*tan2phi );
        double x1 = (a*coslam)/tmpden + h*coslam*cosphi;
        double y1 = (a*sinlam)/tmpden + h*sinlam*cosphi;
        double tmp3 = sqrt(1 - e*e*sinphi*sinphi);
        double z1 = (a*tmp2*sinphi)/tmp3 + h*sinphi;
        Vector3 p1(x1,y1,z1);
        Vector3 p2(init_ecef_x, init_ecef_y, init_ecef_z); //
        Vector3 posDiff = p1 - p2;
        Vector3 orgLLH(init_x, init_y, init_z);
        double sinPhi = sin(orgLLH(0));
        double cosPhi = cos(orgLLH(0));
        double sinLam = sin(orgLLH(1));
        double cosLam = cos(orgLLH(1));
        Matrix R_dist = ( Matrix(3,3) << (-1*sinLam), cosLam, 0, ((-1*sinPhi)*cosLam), ((-1*sinPhi)*sinLam), cosPhi, (cosPhi*cosLam), (cosPhi*sinLam), sinPhi ).finished();
        Vector3 pos;
        pos = R_dist*posDiff;
        return pos;

}
CoreNav::Vector3 CoreNav::llh_to_xyz(double lat, double lon, double height){


        double phi = lat;
        double lambda = lon;
        double h = height;

        double a = 6378137.0000;  //% earth semimajor axis in meters
        double b = 6356752.3142;  //% earth semiminor axis in meters
        double e = sqrt(1-pow((b/a),2));

        double sinphi = sin(phi);
        double cosphi = cos(phi);
        double coslam = cos(lambda);
        double sinlam = sin(lambda);
        double tan2phi = pow((tan(phi)),2);
        double tmp2 = 1 - e*e;
        double tmpden = sqrt( 1 + tmp2*tan2phi );
        double x1 = (a*coslam)/tmpden + h*coslam*cosphi;
        double y1 = (a*sinlam)/tmpden + h*sinlam*cosphi;
        double tmp3 = sqrt(1 - e*e*sinphi*sinphi);
        double z1 = (a*tmp2*sinphi)/tmp3 + h*sinphi;
        Vector3 ecef_xyz(x1,y1,z1);

        return ecef_xyz;

}
double CoreNav::derivcost(double z, double delta){
        if(std::abs(z) > delta){
          if (z > 0) {
            return delta;
          }
          else if (z < 0){
            return -1*delta;
          }
          else{
            return 0.0;
          }
        }
        else {
          return z;
        }
}

// ROBUST FILTERS
void CoreNav::normalUpdate(Eigen::RowVectorXd res){

        // Standard filter
        ROS_INFO_ONCE("Running normal update equations .. ");
        K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+R_).inverse();
        error_states_ = error_states_+K_*res.transpose();
        P_= (Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * R_ * K_.transpose();

}
void CoreNav::UpdateR(Eigen::RowVectorXd res){

      // Incremental  Covariance Estimation  -- Ryan
        std::vector<mixtureComponents> globalMixtureModelLatest = iceclass.globalMixtureModel;

        int ind(0);
        double prob, probMax(0.0);
        Eigen::MatrixXd cov_min(4,4);     //TODO: is the size okay ?
        Eigen::RowVectorXd mean_min(4);

        for(int k=0; k<globalMixtureModelLatest.size();k++){

            merge::mixtureComponents mixtureComp = globalMixtureModelLatest[k];

            auto cov = mixtureComp.get<4>();

            Eigen::RowVectorXd mean = mixtureComp.get<3>();

            double quadform  = (res-mean)* (mixtureComp.get<4>()).inverse() * (res-mean).transpose();
            double norm = std::pow(std::sqrt(2 * M_PI),-1) * std::pow((mixtureComp.get<4>()).determinant(), -0.5);

            prob = (norm) * exp(-0.5 * quadform);  // select best component

            if (prob >= probMax)
            {
                    ind = k;
                    probMax = prob;
                    cov_min = mixtureComp.get<4>();
                    mean_min = mixtureComp.get<3>();
            }
          }

      R_ = cov_min;
      ice_mean = mean_min;
      iceclass.all_res_count +=1 ;
      Eigen::RowVectorXd resdiff(4);
      resdiff = res-ice_mean;

      // checking for inlier or outlier
      double mahalcost(0.0);
      mahalcost = resdiff*(H_*P_*H_.transpose()+R_).inverse()*resdiff.transpose();
      double critical_valchi4 = 9.488;

      double scale = mahalcost/critical_valchi4;

      if (scale < 1.0){
          R_ = cov_min;
          iceclass.icefunc(res,ind);
          iceclass.is_outlier = false;
      }
      // inflate  the R_ matrix if outlier
      else{
          R_ = cov_min;
          //R_ = (scale -1)*H_*P_*H_.transpose()+ scale*cov_min;
          iceclass.res_count +=1 ;
          iceclass.merging(res);
          iceclass.is_outlier = true;
      }
}
void CoreNav::huberUpdate(){

  //Huber filter (Karlgraad) Non-linear Regression HUber-Kalman Filtering and Fixed-Interval Smoothing

        Eigen::MatrixXd epCov(19,19);
        epCov.setZero(19,19);

        epCov.block<15,15>(0,0) = P_;
        epCov.block<4,4>(15,15) = R_;

        Eigen::LLT<MatrixXd> lltofcov(epCov);

        Eigen::MatrixXd S = lltofcov.matrixL();

        Eigen::MatrixXd Sinv  = S.inverse();

        Eigen::MatrixXd F(19,1);

        F.block<15,1>(0,0) = error_states_;
        F.block<4,1>(15,0) = Z_;

        Eigen::MatrixXd Y = Sinv*F;

        Eigen::MatrixXd X(19,15);

        X.block<15,15>(0,0) = Eigen::MatrixXd::Identity(15,15);
        X.block<4,15>(15,0) = H_;

        X = Sinv*X;

        typedef Eigen::Matrix<double, 19, 1> Vector19d;
        Vector19d omeg;

        double delta = 1.5;

        Eigen::VectorXd error_states_prev = error_states_;
        double diff(100.0);
        error_states_prev =  ((X.transpose()*X).inverse())*(X.transpose()*Y);
        DiagonalMatrix<double, 19 > M;
        int iter(0);
        double weight(1);
        while ((diff > 0.000001) && (iter <10)){                              // iterative re-weighted least squares

        for (int i = 0; i < 19; i++){
          if ( (Y(i) - X.row(i)*error_states_) != 0.0 ){
            //if ( i > 14){  // Only thresholding the measurements
              omeg(i) = derivcost((Y(i) - X.row(i)*error_states_prev), delta)/(Y(i) - X.row(i)*error_states_);
            //  }
            }
          else{
            omeg(i) = 1;
            //cout << "Omeg is 1" << endl;
          }
        }

        M = omeg.asDiagonal();
        error_states_ = ((X.transpose()*M*X).inverse())*(X.transpose()*M*Y);
        iter = iter +1;
        //cout << "Iteration Number " << iter << endl;

        diff = (error_states_ - error_states_prev).norm();
        error_states_prev = error_states_;
      }

      if ((diff < 0.000001)|| (iter >= 10)){
        //ROS_INFO("converged --- Going to next epoch");
      }

      P_ = (X.transpose()*M*X).inverse();
}
void CoreNav::adaptiveUpdate(Eigen::RowVectorXd res){

      //res = res - ice_mean;
      double mahalcost(0.0);
      mahalcost = res*(H_*P_*H_.transpose()+R_).inverse()*res.transpose();
      double critical_valchi4 = 9.488;

      double scale = mahalcost/critical_valchi4;

      if (scale < 1.0){
        normalUpdate(res);
        //huberUpdate();
      }
      // else{
      //   continue;
        // MatrixXd R_post(4,4);
        // R_post = (scale -1)*H_*P_*H_.transpose()+ scale*R_;
        // R_ = R_post;
        //normalUpdate(res);
        //huberUpdate();

        // K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+ R_post).inverse();
        // error_states_ = error_states_+K_*res.transpose();
        // P_= (Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * R_post * K_.transpose();
      // }
}
void CoreNav::orkf1Update(Eigen::RowVectorXd res){

      //iid noise Variational Filter by Agememnoni An outlier robust Kalman Filter

      ROS_INFO_ONCE(" Running variational filter ");
      double likelihood = 0.0;
      int dof1 = 250;
      //Matrix4f cov = R_.cast<float>();
      // cov << 1, 0, 0, 0,
      //        0, 1, 0, 0,
      //        0, 0, 1, 0,
      //        0, 0, 0, 1;
      //sampleR(s,cov);
      MatrixXd R_post(4,4);
      //R_post = R_sample;
      // R_post = (K_k -1)*H_*P_*H_.transpose()+ K_k*R_;
      for (int i = 0; i < 5; i++){

          res = Z_-H_*error_states_ ;
          R_post = (dof1*R_ + res.transpose()*res + H_*P_*H_.transpose())/(dof1+1);

          K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+R_post).inverse();
          error_states_ = error_states_+K_*(Z_-H_*error_states_);
          P_= (Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * R_post * K_.transpose();

          // double quad = res*R_post.inverse()*res.transpose();
          // likelihood = std::pow((1 + quad/dof), -(dof+1)/2);
          //
          // std::cout << "Likelihood value --  " << likelihood << endl;
      }

      cout << " Done all iterations " << endl;

  }
void CoreNav::orkf1DriftUpdate(Eigen::RowVectorXd res){

        // Drifting noise Variational Filter by Agememnoni An outlier robust Kalman Filter

        ROS_INFO_ONCE(" Running variational filter ");
        // double likelihood = 0.0;
        //double threshold = 0.9;
        double rho; //forgetting factor

        // if (rearVel_ > 0){
        //   rho = 0.00001;
        // }
        // else{
        rho = 0.0000001;
        // }

        int d = 4;

        for (int i = 0; i < 5; i++){

            res = Z_-H_*error_states_ ;

            R_drift = (1 -rho)*R_drift_prev + res.transpose()*res + H_*P_*H_.transpose();
            dof = (1 - rho)*dof_prev + rho*(d-1) + 1;

            K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+ (R_drift/dof)).inverse();
            error_states_ = error_states_+K_*(Z_-H_*error_states_);
            P_= (Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * (R_drift/dof) * K_.transpose();

            // double quad = res*(R_drift/dof).inverse()*res.transpose();
            // likelihood = std::pow((1 + quad/dof), -(dof+1)/2);

            std::cout << " dof --  " << dof << endl;
        }

        cout << " Done all iterations " << endl;
        R_drift_prev = R_drift;
        dof_prev = dof;

 }
void CoreNav::orkf2Update(Eigen::RowVectorXd res){

        //Variational Filter by Sarkka Recursive outlier-robust filtering and smoothing for nonlinear systems using the multivariate Student-t distribution
        int L = 15;
        double alpha = 1; //1e-3< alpha <= 1
        double beta = 2;
        double k = 0;
        double lambda = std::pow(alpha,2)*(L + k) - L;
        double gamma = std::pow(L + lambda, 0.5);
        double w0_m = lambda/(L+lambda);
        double w0_c = w0_m + (1 - std::pow(alpha,2) + beta);
        double neu = 300; // parameter for gamma prior for lambda
        double d = 4;
        int N = 5;
        int n = 15; //Dimension of the state
        double lambda2 = 1;

        for(int j = 0; j < N; j++){

          std::vector<CoreNav::Vector15> sigma_pts;
          sigma_pts.push_back(error_states_);

          std::vector<int> weights_m(31,1/(2*(L+lambda)));
          std::vector<int> weights_c(31,1/(2*(L+lambda)));
          weights_m[0] = w0_m;
          weights_c[0] = w0_c;

          K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+ (1/lambda2)*R_).inverse();
          error_states_ = error_states_+K_*(Z_-H_*error_states_);
          P_= (Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * R_ * K_.transpose();

          CoreNav::Matrix P_sqrt(15,15);
          cout << " Calculating square root ... " << endl;
          Eigen::LLT<MatrixXd> lltofcov(P_);
          Eigen::MatrixXd S = lltofcov.matrixL();
          P_sqrt = gamma*S; //calculate the square root of the covariance matrix
          cout << " Done " << endl;
          // cout << P_sqrt << endl;
          for(int j = 0; j < 15; j++){
            //cout << j << endl;

            CoreNav::Vector15 sig_error_state1, sig_error_state2;

            sig_error_state1 = error_states_ + P_sqrt.col(j);
            sig_error_state2 = error_states_ - P_sqrt.col(j);

            sigma_pts.push_back(sig_error_state1);
            sigma_pts.push_back(sig_error_state2);
          }

          cout << "Done with pushbacks " <<  endl;
          // now calculate the expected value of the measurement noise matrix with respect to the posterior distribution
          MatrixXd R_post(4,4);
          R_post << 0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0,
          0.0,0.0,0.0,0.0;

          cout <<" calculating R " << endl;
          for(int i = 0; i < sigma_pts.size(); i++){
            // cout << i << endl;
            res = Z_-H_*sigma_pts[i];
            R_post = R_post + weights_c[i]*res.transpose()*res;
          }

          // cout << R_post << endl;
          lambda2 = (neu + d )/(neu + (R_post*R_.inverse()).trace());
          cout << "Calculating lambda - " << lambda2 << endl;

        }

}
void CoreNav::orkf3Update(Eigen::RowVectorXd res){

      //iid noise Variational Filter by Agememnoni An outlier robust Kalman Filter

      ROS_INFO_ONCE(" Running variational filter ");
      // int n = 15;
      int m = 4;

      if (rearVel_ > 0){
        rho = 0.99999;
      }
      else{
        rho = 1;
      }

      u = rho*(u-m-1) + m + 1;
      U = rho*U;
      CoreNav::Matrix A;
      MatrixXd R_post(4,4);
      Eigen::Matrix<double, 4, 4> B;

      // if (rearVel_ > 0){
      tau = 2000;
      // }
      // else{
      //   tau = 10;
      // }

      t = num_states_ + tau +1;
      T = tau*P_;

      CoreNav::Vector15 error_states_post = error_states_;

      for (int i = 0; i < 3; i++){  //variational iteration

          A = P_ + (error_states_post - error_states_)*(error_states_post - error_states_).transpose();
          t = t + 1;
          T = A + T;
          B = (Z_-H_*error_states_post)*(Z_-H_*error_states_post).transpose() + H_*P_*H_.transpose();
          u = u + 1;
          U = B + U;
          P_ = T/(t - num_states_ -1);
          R_post = U/(u -m -1);
          K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+R_post).inverse();
          error_states_post = error_states_+K_*(Z_-H_*error_states_);
          P_= (Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * R_post* K_.transpose();
      }

      error_states_ = error_states_post;
      cout << " Done all iterations " << endl;


  }
void CoreNav::orkf4Update(Eigen::RowVectorXd res){

        //Another scaling filter by G Chang  Robust Kalman filtering based on Mahalanobis distance as outlier judging criterion

        double mahalcost(0.0);
        mahalcost = res*(H_*P_*H_.transpose()+R_).inverse()*res.transpose();
        double critical_valchi4 = 9.488;

        double K_k;
        double scale = mahalcost/critical_valchi4;

        if (scale < 1.0){
            K_k = 1.0;
        }
        else{
          K_k = scale;
        }

        MatrixXd R_post(4,4);
        R_post = (K_k -1)*H_*P_*H_.transpose()+ K_k*R_;

        K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+ R_post).inverse();
        error_states_ = error_states_+K_*res.transpose();
        P_= (Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * R_post * K_.transpose();

}
void CoreNav::orkf5Update(Eigen::RowVectorXd res){

        // Adaptive filter by Akhlaghi  https://arxiv.org/pdf/1702.00884.pdf
        double alpha = 0.05;  //fading factor

        K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+R_).inverse();

        error_states_ = error_states_+K_*res.transpose();

        Eigen::RowVectorXd residual(4);

        residual = Z_-H_*error_states_ ;

        R_ = alpha*R_ + (1 - alpha)*(residual.transpose()*residual +  H_*P_*H_.transpose());
        K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+R_).inverse();
        P_= (Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * R_ * K_.transpose();
        //Q_ = alpha*Q_ + (1-alpha)*K_*res.transpose()*res*K_.transpose();
}
void CoreNav::orkf6Update(Eigen::RowVectorXd res){

        //Covariance Scaling filter by Yang Adaptively robust filtering for kinematic geodetic positioning

        ROS_INFO_ONCE(" Running covariance-scaling filter ");
        MatrixXd R_post(4,4);
        R_post = R_;
        double delta = 1.5;

        for (int i=0; i < 4; i++){
            if (std::abs(res(i)/std::pow(R_(i,i),0.5)) <= delta){
                R_post(i,i) = R_(i,i);
            }
            else {
              R_post(i,i) = R_(i,i)*(delta/std::abs(res(i)/std::pow(R_(i,i),0.5)));
            }
        }

        K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+R_post).inverse();
        error_states_ = error_states_+K_*res.transpose();
        P_= (Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * R_post * K_.transpose();

}

// CALLBACKS

bool CoreNav::RegisterCallbacks(const ros::NodeHandle& n){
        // Create a local nodehandle to manage callback subscriptions.
        ros::NodeHandle nl(n);

        position_pub_ = nl.advertise<geometry_msgs::PointStamped>( "position", 10, false);
        velocity_pub_ = nl.advertise<geometry_msgs::PointStamped>( "velocity", 10, false);
        attitude_pub_ = nl.advertise<geometry_msgs::PointStamped>( "attitude", 10, false);
        ins_xyz_pub_=nl.advertise<geometry_msgs::PointStamped>( "ecef", 10, false);
        pos_llh_pub_=nl.advertise<geometry_msgs::PointStamped>( "llh", 10, false);
        enu_pub_ = nl.advertise<nav_msgs::Odometry>( "enu", 10, false);
        enu_pub2_ = nl.advertise<nav_msgs::Odometry>( "enu2", 10, false);
        cn_pub_=nl.advertise<nav_msgs::Odometry>("cn_odom", 10, false);
        slip_pub_=nl.advertise<geometry_msgs::PointStamped>( "slip", 10, false);
        res_pub_ = nl.advertise<std_msgs::Float32MultiArray>("res",10,false);
        // gp_pub=nl.advertise<core_nav::GP_Input>("gp_input", 10, false);
        stop_cmd_pub_ = nl.advertise<std_msgs::Float64>("/core_nav/core_nav/stop_cmd", 1);

        imu_sub_ = nl.subscribe(imu_topic_,  1, &CoreNav::ImuCallback, this);
        odo_sub_ = nl.subscribe(odo_topic_,  1, &CoreNav::OdoCallback, this);
        joint_sub_ = nl.subscribe(joint_topic_,  1, &CoreNav::JointCallBack, this);
        cmd_sub_ = nl.subscribe(cmd_topic_,  1, &CoreNav::CmdCallBack, this);
        encoderLeft_sub_ = nl.subscribe(encoderLeft_topic_,  1, &CoreNav::EncoderLeftCallBack, this);
        encoderRight_sub_ = nl.subscribe(encoderRight_topic_,  1, &CoreNav::EncoderRightCallBack, this);
        // gp_sub_ =nl.subscribe(gp_topic_,1, &CoreNav::GPCallBack, this);
        return true;
}
void CoreNav::OdoCallback(const OdoData& odo_data_){
        odo = getOdoData(odo_data_);
        // ROS_INFO("Odometry called\n");
        has_odo_ = true; // ODOMETRY ON/OFF
        if (first_odo_)
        {
                // ROS_INFO("First Odometry\n");
                odo_stamp_curr_ = (odo_data_.header.stamp).toSec();
                first_odo_ = false;
        }
        else{
                odo_stamp_prev_ = odo_stamp_curr_;
                odo_stamp_curr_ = (odo_data_.header.stamp).toSec();
                // ROS_INFO("Update Flag is set to true\n");

        }
        return;
}
void CoreNav::JointCallBack(const JointData& joint_data_){
  joint = getJointData(joint_data_);
  has_joint_ = true;
  if (first_joint_)
  {
          joint_stamp_curr_ = (joint_data_.header.stamp).toSec();
          first_joint_ = false;
  }
  else{
    joint_stamp_prev_ = joint_stamp_curr_;
    joint_stamp_curr_ = (joint_data_.header.stamp).toSec();
      Update();

  }
  return;
}
void CoreNav::ImuCallback(const ImuData& imu_dataAdis_){
        imu = getImuData(imu_dataAdis_);
        has_imu_ = true;
        if (first_imu_)
        {
                imu_stamp_curr_ = (imu_dataAdis_.header.stamp).toSec();
                first_imu_ = false;
        }
        else{
                imu_stamp_prev_  = imu_stamp_curr_;
                imu_stamp_curr_ = (imu_dataAdis_.header.stamp).toSec();
                imuTime  = imu_dataAdis_.header.stamp;

                //Propagate(imu,odo,cmd,encoderLeft,encoderRight,joint);
                //         std::cout << "rearVel1_" << '\n';
                //ROS_INFO("imu callback, accel_x = %f",imu_dataAdis_.linear_acceleration.x);
                //ROS_INFO("imu_stamp_curr_ = %f, prev = %f",imu_stamp_curr_, imu_stamp_prev_);

                Propagate();

        }
        return;
}
void CoreNav::CmdCallBack(const CmdData& cmd_data_){
        cmd = getCmdData(cmd_data_);
        has_cmd_ = true;
        return;
}
void CoreNav::EncoderLeftCallBack(const EncoderLeftData& encoderLeft_data_){
        encoderLeft = getEncoderLeftData(encoderLeft_data_);
        has_encoderLeft_ = true;
        return;
}
void CoreNav::EncoderRightCallBack(const EncoderRightData& encoderRight_data_){
        encoderRight = getEncoderRightData(encoderRight_data_);
        has_encoderRight_ = true;
        return;
}
CoreNav::Vector13 CoreNav::getOdoData(const OdoData& odo_data_){

CoreNav::Vector13 odoVec( (Vector(13) << odo_data_.pose.pose.position.x,
                                   odo_data_.pose.pose.position.y,
                                   odo_data_.pose.pose.position.z,
                                   odo_data_.pose.pose.orientation.w,
                                   odo_data_.pose.pose.orientation.x,
                                   odo_data_.pose.pose.orientation.y,
                                   odo_data_.pose.pose.orientation.z,
                                   odo_data_.twist.twist.linear.x,
                                   odo_data_.twist.twist.linear.y,
                                   odo_data_.twist.twist.linear.z,
                                   odo_data_.twist.twist.angular.x,
                                   odo_data_.twist.twist.angular.y,
                                   odo_data_.twist.twist.angular.z ).finished() );

        return odoVec;
}
CoreNav::Vector4 CoreNav::getJointData(const JointData& joint_data_){
        CoreNav::Vector4 jointVec((Vector(4) << joint_data_.velocity[0],
                                   joint_data_.velocity[1],
                                   joint_data_.velocity[2],
                                   joint_data_.velocity[3] ).finished());
        return jointVec;
}
CoreNav::Vector6 CoreNav::getImuData(const ImuData& imu_dataAdis_){
        CoreNav::Vector6 imuVec((Vector(6) << imu_dataAdis_.linear_acceleration.x,
                                 imu_dataAdis_.linear_acceleration.y,
                                 imu_dataAdis_.linear_acceleration.z,
                                 imu_dataAdis_.angular_velocity.x,
                                 imu_dataAdis_.angular_velocity.y,
                                 imu_dataAdis_.angular_velocity.z).finished());
        return imuVec;
}
CoreNav::Vector CoreNav::getCmdData(const CmdData& cmd_data_){
        CoreNav::Vector3 cmdVec((Vector(3) << cmd_data_.linear.x,
                                   cmd_data_.linear.y,
                                   cmd_data_.linear.z).finished());
        if(gp_flag)
        {
                if(fabs(cmd_data_.linear.x) < 0.0001)
                {
                        started_driving_again_flag = false;
                        ROS_INFO_ONCE("set started driving again flag to false");
                }
        }
        if(started_driving_again_flag==false)
        {
                if(fabs(cmd_data_.linear.x) > 0.0001)
                {
                        started_driving_again_flag = true;
                        gp_flag = false;
                        ROS_INFO_ONCE("set GP flag back to false");
                }
        }
        return cmdVec;
}
CoreNav::Vector2 CoreNav::getEncoderLeftData(const EncoderLeftData& encoderLeft_data_){
        CoreNav::Vector2 encoderLeftVec((Vector(2) << encoderLeft_data_.encoder_counter_absolute[0],
                                   encoderLeft_data_.encoder_counter_absolute[1]).finished());
        return encoderLeftVec;
}
CoreNav::Vector2 CoreNav::getEncoderRightData(const EncoderRightData& encoderRight_data_){
        CoreNav::Vector2 encoderRightVec((Vector(2) << encoderRight_data_.encoder_counter_absolute[0],
                                   encoderRight_data_.encoder_counter_absolute[1]).finished());
        return encoderRightVec;
}
// PUBLISHERS
void CoreNav::PublishStates(const CoreNav::Vector3& states,const ros::Publisher& pub){
        // // Check for subscribers before doing any work.
        if(pub.getNumSubscribers() == 0)
                return;

        geometry_msgs::PointStamped msg;

        msg.point.x = states(0);
        msg.point.y = states(1);
        msg.point.z = states(2);
        msg.header.frame_id = frame_id_fixed_;
        msg.header.stamp = ros::Time::now();

        pub.publish(msg);
}
void CoreNav::PublishStateswCov(const CoreNav::Vector6& states,const ros::Publisher& pub){
        // // Check for subscribers before doing any work.
        if(pub.getNumSubscribers() == 0)
                return;

        nav_msgs::Odometry CNmsg;
        CNmsg.twist.twist.linear.x= states(3); // storing the error states in the velocity fields
        CNmsg.twist.twist.linear.y= states(4);
        CNmsg.twist.twist.linear.z= states(5);
        CNmsg.pose.pose.position.x= states(0);
        CNmsg.pose.pose.position.y= states(1);
        CNmsg.pose.pose.position.z= states(2);
        Eigen::MatrixXd covar(6,6);
        covar = P_.block(3,3,6,6);
        //cout << covar << endl; //block<p,q>(i,j); //covariance of att,vel,pos; block size p,q starting at i,j
        // Map<RowVectorXd> v1(covar.data(), covar.size());
        // boost::array<double, 36> B(Map<boost::array<double, 36>v1);
        // Matrix<float,Dynamic,Dynamic,RowMajor> M2(M1);
        // Map<RowVectorXf> v2(M2.data(), M2.size());
        int k = 0;
        for (int i = 0; i<6; i++){
          for (int j=0; j<6; j++){
            CNmsg.pose.covariance[k] = covar(j,i);
            k = k+1;
          }
        }

        //CNmsg.pose.covariance = B;

        CNmsg.header.frame_id = frame_id_fixed_;
        CNmsg.header.stamp = imuTime;

        pub.publish(CNmsg);
}
void CoreNav::PublishStatesenu(const CoreNav::Vector9& states,const ros::Publisher& pub){
        // // Check for subscribers before doing any work.
        if(pub.getNumSubscribers() == 0)
                return;

        nav_msgs::Odometry CNmsg;
        CNmsg.twist.twist.linear.x= states(3); // storing the error states in the velocity fields
        CNmsg.twist.twist.linear.y= states(4);
        CNmsg.twist.twist.linear.z= states(5);
        CNmsg.pose.pose.position.x= states(6);
        CNmsg.pose.pose.position.y= states(7);
        CNmsg.pose.pose.position.z= states(8);
        // Eigen::MatrixXd covar(6,6);
        // covar = P_.block(3,3,6,6);
        //cout << covar << endl; //block<p,q>(i,j); //covariance of att,vel,pos; block size p,q starting at i,j
        // Map<RowVectorXd> v1(covar.data(), covar.size());
        // boost::array<double, 36> B(Map<boost::array<double, 36>v1);
        // Matrix<float,Dynamic,Dynamic,RowMajor> M2(M1);
        // Map<RowVectorXf> v2(M2.data(), M2.size());
        //CNmsg.pose.covariance = B;

        CNmsg.header.frame_id = frame_id_fixed_;
        CNmsg.header.stamp = ros::Time::now();

        pub.publish(CNmsg);
}
void CoreNav::PublishStatesSlip(const CoreNav::Vector3& slip_states,const ros::Publisher& slip_pub){
        // // Check for subscribers before doing any work.
        if(slip_pub.getNumSubscribers() == 0)
                return;

        geometry_msgs::PointStamped msg;

        msg.point.x = slip_states(0); //slip
        msg.point.y = slip_states(1); //rearWheelLinearVelocity
        msg.point.z = slip_states(2); //INSestimatedLinearVelocityX
        msg.header.frame_id = frame_id_fixed_;
        msg.header.stamp = ros::Time::now();

        slip_pub.publish(msg);
}
void CoreNav::PublishStatesCN(const CoreNav::Vector9& cn_states,const ros::Publisher& cn_pub_){
        // // Check for subscribers before doing any work.
        if(cn_pub_.getNumSubscribers() == 0)
                return;

        nav_msgs::Odometry CNmsg;

        tf::Matrix3x3 obs_mat;
        obs_mat.setEulerYPR(-cn_states(2),cn_states(1),cn_states(0));

        tf::Quaternion q_tf;
        obs_mat.getRotation(q_tf);

        CNmsg.pose.pose.position.x=cn_states(7);
        CNmsg.pose.pose.position.y=-cn_states(6);
        CNmsg.pose.pose.position.z=-cn_states(8);
        CNmsg.pose.pose.orientation.w=q_tf.getW();
        CNmsg.pose.pose.orientation.x=q_tf.getX();
        CNmsg.pose.pose.orientation.y=q_tf.getY();
        CNmsg.pose.pose.orientation.z=q_tf.getZ();
        CNmsg.twist.twist.linear.x=cn_states(3);
        CNmsg.twist.twist.linear.y=-cn_states(4);
        CNmsg.twist.twist.linear.z=-cn_states(5);
        CNmsg.header.frame_id = frame_id_fixed_;
        CNmsg.child_frame_id = frame_id_imu_;
        CNmsg.header.stamp = ros::Time::now();

        cn_pub_.publish(CNmsg);
}
void CoreNav::PublishRes(Eigen::RowVectorXd res,const ros::Publisher& pub){
        // // Check for subscribers before doing any work.
        if(pub.getNumSubscribers() == 0)
                return;

        std_msgs::Float32MultiArray msg;
        msg.data.clear();
        for(int i =0; i <4; i++){
          msg.data.push_back(res(i));
        }
        pub.publish(msg);
}
// INITIALIZATION
bool CoreNav::LoadParameters(const ros::NodeHandle& n){
        // Load frame ids.
        if(!pu::Get("frames/frame_id_out", frame_id_out_)) return false;
        if(!pu::Get("frames/frame_id_imu", frame_id_imu_)) return false;
        if(!pu::Get("frames/frame_id_odo", frame_id_odo_)) return false;
        if(!pu::Get("frames/frame_id_fixed", frame_id_fixed_)) return false;

        // Load topics
        if(!pu::Get("imu/topic", imu_topic_)) return false;
        if(!pu::Get("odo/topic", odo_topic_)) return false;
        if(!pu::Get("joint/topic", joint_topic_)) return false;
        if(!pu::Get("cmd/topic", cmd_topic_)) return false;
        if(!pu::Get("encoderLeft/topic", encoderLeft_topic_)) return false;
        if(!pu::Get("encoderRight/topic", encoderLeft_topic_)) return false;
        if(!pu::Get("gp/topic", gp_topic_)) return false;
        if(!pu::Get("res/topic", res_topic_)) return false;
        // Load update rate
        if(!pu::Get("imu/publish_hz", publish_hz_)) return false;
        if(!pu::Get("imu/sensor_pub_rate", sensor_pub_rate_)) return false;

        // Load imu noise specs
        if(!pu::Get("imu/noise/sig_gyro_inRun", sig_gyro_inRun_)) return false;
        if(!pu::Get("imu/noise/sig_ARW", sig_ARW_)) return false;
        if(!pu::Get("imu/noise/sig_accel_inRun", sig_accel_inRun_)) return false;
        if(!pu::Get("imu/noise/sig_VRW", sig_VRW_)) return false;

        // Load initial position and orientation.
        if (!pu::Get("init_llh/x", init_x)) return false;
        if (!pu::Get("init_llh/y", init_y)) return false;
        if (!pu::Get("init_llh/z", init_z)) return false;
        if (!pu::Get("init/velocity/x", init_vx)) return false;
        if (!pu::Get("init/velocity/y", init_vy)) return false;
        if (!pu::Get("init/velocity/z", init_vz)) return false;
        if (!pu::Get("init_att/x", init_roll)) return false;
        if (!pu::Get("init_att/y", init_pitch)) return false;
        if (!pu::Get("init_att/z", init_yaw)) return false;
        if (!pu::Get("init_ecef/x", init_ecef_x)) return false;
        if (!pu::Get("init_ecef/y", init_ecef_y)) return false;
        if (!pu::Get("init_ecef/z", init_ecef_z)) return false;

        // Load initial covariance of state vector estimate
        if (!pu::Get("init/position/covx", init_cov_x)) return false;
        if (!pu::Get("init/position/covy", init_cov_y)) return false;
        if (!pu::Get("init/position/covz", init_cov_z)) return false;
        if (!pu::Get("init/velocity/covx", init_cov_vx)) return false;
        if (!pu::Get("init/velocity/covy", init_cov_vy)) return false;
        if (!pu::Get("init/velocity/covz", init_cov_vz)) return false;
        if (!pu::Get("init/orientation/covx", init_cov_roll)) return false;
        if (!pu::Get("init/orientation/covy", init_cov_pitch)) return false;
        if (!pu::Get("init/orientation/covz", init_cov_yaw)) return false;

        if (!pu::Get("mean_bias_a/x", init_bias_mean_a_x)) return false;
        if (!pu::Get("mean_bias_a/y", init_bias_mean_a_y)) return false;
        if (!pu::Get("mean_bias_a/z", init_bias_mean_a_z)) return false;
        if (!pu::Get("mean_bias_g/x", init_bias_mean_g_x)) return false;
        if (!pu::Get("mean_bias_g/y", init_bias_mean_g_y)) return false;
        if (!pu::Get("mean_bias_g/z", init_bias_mean_g_z)) return false;

        if (!pu::Get("init/std_bias_a/x", init_bias_std_a_x)) return false;
        if (!pu::Get("init/std_bias_a/y", init_bias_std_a_y)) return false;
        if (!pu::Get("init/std_bias_a/z", init_bias_std_a_z)) return false;
        if (!pu::Get("init/std_bias_g/x", init_bias_std_g_x)) return false;
        if (!pu::Get("init/std_bias_g/y", init_bias_std_g_y)) return false;
        if (!pu::Get("init/std_bias_g/z", init_bias_std_g_z)) return false;

        if (!pu::Get("wheel/T_r_", T_r_)) return false;
        if (!pu::Get("wheel/s_or_", s_or_)) return false;
        if (!pu::Get("wheel/s_delta_or_", s_delta_or_)) return false;

        // ROS_INFO("Loaded Parameters\n");
        return true;
}
bool CoreNav::Initialize(const ros::NodeHandle& n){
        name_ = ros::names::append(n.getNamespace(), "CoreNav");

        if(!LoadParameters(n)) {
                ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
                return false;
        }

        if(!Init(n)) {
                ROS_ERROR("%s: Failed to initialize.", name_.c_str());
                return false;
        }

        if(!RegisterCallbacks(n)) {
                ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
                return false;
        }
        new_gp_data_arrived_ = false;

        return true;
}
bool CoreNav::Init(const ros::NodeHandle& n){

  error_states_ = Eigen::VectorXd::Zero(num_states_);

          // // Construct initial P matrix, Diagonal P
          //dof = 25;
          dof_prev = 250;
          P_= Eigen::MatrixXd::Zero(num_states_,num_states_);
          P_pred=Eigen::MatrixXd::Zero(num_states_,num_states_);

          Eigen::MatrixXd P= Eigen::MatrixXd::Zero(15,15);
          P(0,0)=init_cov_roll;       P(1,1)=init_cov_pitch;      P(2,2)=init_cov_yaw;
          P(3,3)=init_cov_vx;         P(4,4)=init_cov_vy;         P(5,5)=init_cov_vz;
          P(6,6)=init_cov_x;          P(7,7)=init_cov_y;          P(8,8)=init_cov_z;
          P(9,9)=init_bias_std_a_x;   P(10,10)=init_bias_std_a_y; P(11,11)=init_bias_std_a_z;
          P(12,12)=init_bias_std_g_x; P(13,13)=init_bias_std_g_y; P(14,14)=init_bias_std_g_z;

          ba_(0) = P(9,9);   ba_(1) = P(10,10); ba_(2) = P(11,11);
          bg_(0) = P(12,12); bg_(1) = P(13,13); bg_(2) = P(14,14);

          // ODOMETRY R VALUES
          P_=P;
          P_pred=P_;

          R_1<<0.5,0.5,0.0,0.0,
          1/T_r_, -1/T_r_, 0.0,0.0,
          0.0, 0.0,1.0,0.0,
          0.0,0.0,0.0,1.0;

          R_2<<0.03*0.03, 0.0,0.0,0.0,
          0.0, 0.03*0.03, 0.0,0.0,
          0.0,0.0,0.05*0.05,0.0,
          0.0,0.0,0.0,0.05*0.05;

          R_<< 25*R_1*R_2*R_1.transpose();
          //R_drift = dof*R_;
          R_drift_prev = dof_prev*R_;
          u  = 2000; // try higher than 500 or try with higher tau
          U = R_*(u - 5);  //R_*(u -m -1)
          //T = tau*P_;
          tau = 500;   //VBAKF [ paper suggests 2 -6 ]
          rho = 0.999; //VBAKF
          // int n = 15; // dimension of state
          //int m = 4;  // dimension of measuremnet
          // t = num_states_ + tau +1;

          // ZERO UPDATES R VALUES
          R_zero << std::pow(0.01,2),0,0,0,0,0,
                    0,std::pow(0.01,2),0,0,0,0,
                    0,0,std::pow(0.0025,2),0,0,0,
                    0,0,0,std::pow(0.02,2),0,0,
                    0,0,0,0,std::pow(0.02,2),0,
                    0,0,0,0,0,std::pow(1.0,2);
          R_zero = R_zero;

          // NON HOLONONOMIC R VALUES
          R_holo << 0.05,0,
                    0,0.1;
          R_holo = R_holo;


          H11_<< 0.0,0.0,0.0;
          H12_ << 0.0,0.0,0.0;
          H21_<< 0.0,0.0,0.0;
          H31_<< 0.0,0.0,0.0;
          H32_<< 0.0,0.0,0.0;
          H24_ << 0.0,0.0,0.0;
          H41_ << 0.0,0.0,0.0;
          H42_ << 0.0,0.0,0.0;

          H_zupt << 0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0;

          H_zaru << 0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,
                    0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,
                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1;

          H_zero << 0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,
                    0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,
                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,
                    0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,-1,0,0,0,0,0,0,0,0,0;

          ins_attMinus_ << init_roll, init_pitch, init_yaw;
          ins_velMinus_ << init_vx, init_vy, init_vz;
          ins_posMinus_ << init_x, init_y, init_z;
          psiEst = init_yaw;
          odomUptCount =0;
          startRecording=0;
          stopRecording=0;
          saveCountOdom=0;
          i=0;
          bool first_driving_flag = true;
          return true;
}

// void CoreNav::sampleR(Eigen::Matrix4f covmat){
//
//       Eigen::Rand::Vmt19937_64 urng{ 42 };
//       auto gen3 = Eigen::Rand::makeWishartGen(dof, covmat);
//
//       //generates one sample ( shape (4, 4) )
//       Eigen::Matrix4f sample = gen3.generate(urng);
//       Eigen::Matrix4d sampleD = sample.cast<double>();
//       R_sample = sampleD;
//
// }

// void CoreNav::rodUpdate(){
//   //Ting Robust Outlier Detection
//
// }
