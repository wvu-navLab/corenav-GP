/*
 * Author: Cagri
 */

#include <core_navigation/CoreNav.h>
#include <core_navigation/InsConst.h>
#include <core_navigation/InsUtils.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <geometry_msgs/TransformStamped.h>
#include <hw_interface_plugin_roboteq/RoboteqData.h>
namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

CoreNav::CoreNav() : initialized_(false){
}

CoreNav::~CoreNav(){
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
        P_= Eigen::MatrixXd::Zero(num_states_,num_states_);
        P_pred=Eigen::MatrixXd::Zero(num_states_,num_states_);

        Eigen::MatrixXd P= Eigen::MatrixXd::Zero(15,15);
        P(0,0)=init_cov_roll;     P(1,1)=init_cov_pitch;     P(2,2)=init_cov_yaw;//4.873878716587337e-06;
        P(3,3)=init_cov_vx;       P(4,4)=init_cov_vy;        P(5,5)=init_cov_vz;
        P(6,6)=init_cov_x;        P(7,7)=init_cov_y;         P(8,8)=init_cov_z;
        P(9,9)=init_bias_a_x;     P(10,10)=init_bias_a_y;    P(11,11)=init_bias_a_z;
        P(12,12)=init_bias_g_x;   P(13,13)=init_bias_g_y;    P(14,14)=init_bias_g_z;

        ba_(0) = P(9,9);   ba_(1) = P(10,10); ba_(2) = P(11,11);
        bg_(0) = P(12,12); bg_(1) = P(13,13); bg_(2) = P(14,14);

        // P_pred=P;
        P_=P;
        P_pred=P_;
        
        R_1<<0.5,0.5,0.0,0.0,
        1/T_r_, -1/T_r_, 0.0,0.0,
        0.0, 0.0,1.0,0.0,
        0.0,0.0,0.0,1.0;

        R_2<<0.03*0.03, 0.0,0.0,0.0,
        0.0, 0.03*0.03, 0.0,0.0,
        0.0,0.0,0.05*0.05,0.0,
        0.0,0.0,0.0,1.0;

        R_<<R_1*R_2*R_1.transpose();
        R_IP<<R_;

        R_IP_1 <<0.5,     0.5,    0.0, 0.0,
                1/T_r_,-1/T_r_, 0.0, 0.0,
                0.0,     0.0,     1.0, 0.0,
                0.0,     0.0,     0.0, 1.0;

        R_IP_2<< 0.03*0.03, 0.0,0.0,0.0,
                 0.0, 0.03*0.03,0.0,0.0,
                 0.0, 0.0, 0.05*0.05,0.0,
                 0.0,0.0,0.0,0.05*0.05;

        R_zupt << std::pow(0.02,2),0,0,
        0,std::pow(0.02,2),0,
        0,0,std::pow(0.02,2);

        R_zaru << std::pow(0.01,2),0,0,
        0,std::pow(0.01,2),0,
        0,0,std::pow(0.0025,2);

        R_holo << 0.05,0,0,0.05;

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

        ins_att_ << init_roll, init_pitch, init_yaw;
        ins_vel_ << init_vx, init_vy, init_vz;
        ins_pos_ << init_x, init_y, init_z;
        // ins_enu_ <<0.0,0.0,0.0;
        odomUptCount =0;
        startRecording=0;
        stopRecording=0;
        saveCountOdom=0;
        i=0;
        bool flag = true;
        // new_gp_data_arrived_ = false; // CAGRI: initialize flag to false, because at the beginning, you have not output data from the gp

        return true;
}

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
        if (!pu::Get("init/orientation/x", init_roll)) return false;
        if (!pu::Get("init/orientation/y", init_pitch)) return false;
        if (!pu::Get("init_yaw/z", init_yaw)) return false;
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

        if (!pu::Get("bias_a/x", init_bias_a_x)) return false;
        if (!pu::Get("bias_a/y", init_bias_a_y)) return false;
        if (!pu::Get("bias_a/z", init_bias_a_z)) return false;
        if (!pu::Get("bias_g/x", init_bias_g_x)) return false;
        if (!pu::Get("bias_g/y", init_bias_g_y)) return false;
        if (!pu::Get("bias_g/z", init_bias_g_z)) return false;

        if (!pu::Get("wheel/T_r_", T_r_)) return false;
        if (!pu::Get("wheel/s_or_", s_or_)) return false;
        if (!pu::Get("wheel/s_delta_or_", s_delta_or_)) return false;

        return true;
}

void CoreNav::OdoCallback(const OdoData& odo_data_){
        odo = getOdoData(odo_data_);
        has_odo_ = true; // ODOMETRY ON/OFF
        if (first_odo_)
        {
                odo_stamp_curr_ = (odo_data_.header.stamp).toSec();
                first_odo_ = false;
        }
        else{
                odo_stamp_prev_ = odo_stamp_curr_;
                odo_stamp_curr_ = (odo_data_.header.stamp).toSec();

                update_flag = true;
        }
        return;
}

void CoreNav::JointCallBack(const JointData& joint_data_){
        joint = getJointData(joint_data_);
        has_joint_ = true;
        if (first_joint_)
        {
                joint_stamp_prev_ = (joint_data_.header.stamp).toSec();
                first_joint_ = false;
        }
        joint_stamp_curr_ = (joint_data_.header.stamp).toSec();
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

                //Propagate(imu,odo,cmd,encoderLeft,encoderRight,joint);
                //         std::cout << "rearVel1_" << '\n';
                //ROS_INFO("imu callback, accel_x = %f",imu_dataAdis_.linear_acceleration.x);
                //ROS_INFO("imu_stamp_curr_ = %f, prev = %f",imu_stamp_curr_, imu_stamp_prev_);

                propagate_flag = true;

        }
        return;
}

bool CoreNav::RegisterCallbacks(const ros::NodeHandle& n){
        // Create a local nodehandle to manage callback subscriptions.
        ros::NodeHandle nl(n);

        position_pub_ = nl.advertise<geometry_msgs::PointStamped>( "position", 10, false);
        velocity_pub_ = nl.advertise<geometry_msgs::PointStamped>( "velocity", 10, false);
        attitude_pub_ = nl.advertise<geometry_msgs::PointStamped>( "attitude", 10, false);
        ins_xyz_pub_=nl.advertise<geometry_msgs::PointStamped>( "ecef", 10, false);
        pos_llh_pub_=nl.advertise<geometry_msgs::PointStamped>( "llh", 10, false);
        enu_pub_ = nl.advertise<geometry_msgs::PointStamped>( "enu", 10, false);
        cn_pub_=nl.advertise<nav_msgs::Odometry>("cn_odom", 10, false);
        slip_pub_=nl.advertise<geometry_msgs::PointStamped>( "slip", 10, false);

        gp_pub=nl.advertise<core_nav::GP_Input>("gp_input", 10, false);

        stop_cmd_pub_ = nl.advertise<std_msgs::Float64>("/core_nav/core_nav/stop_cmd", 1);

        imu_sub_ = nl.subscribe(imu_topic_,  10, &CoreNav::ImuCallback, this);
        odo_sub_ = nl.subscribe(odo_topic_,  10, &CoreNav::OdoCallback, this);
        joint_sub_ = nl.subscribe(joint_topic_,  10, &CoreNav::JointCallBack, this);
        cmd_sub_ = nl.subscribe(cmd_topic_,  10, &CoreNav::CmdCallBack, this);
        encoderLeft_sub_ = nl.subscribe(encoderLeft_topic_,  10, &CoreNav::EncoderLeftCallBack, this);
        encoderRight_sub_ = nl.subscribe(encoderRight_topic_,  10, &CoreNav::EncoderRightCallBack, this);
        gp_sub_ =nl.subscribe(gp_topic_,10, &CoreNav::GPCallBack, this);
        return true;
}

void CoreNav::Propagate(const CoreNav::Vector6& imu, const CoreNav::Vector13& odo, const CoreNav::Vector3& cmd,const CoreNav::Vector2& encoderLeft, const CoreNav::Vector2& encoderRight, const CoreNav::Vector4& joint){
        // std::cout << "rearVel_" << '\n';

        dt_imu_ = imu_stamp_curr_ - imu_stamp_prev_;
        //ROS_INFO("dt_imu = %f",dt_imu_);
        count++;
        // Initial bias
        omega_b_ib_ << imu[3] -(init_bias_g_x)-bg_(0), imu[4] - (init_bias_g_y)- bg_(1), imu[5] -(init_bias_g_x) - bg_(2);
        f_ib_b_ << imu[0] -(init_bias_a_x)-ba_(0), imu[1] - (init_bias_a_y)- ba_(1), imu[2] -(-9.81-init_bias_a_z) - ba_(2);
//------------------------------------------------------------------------------
        //Attitude Update---------------------------------------------------------------
        // input =insAtt(:,i-1),omega_ie,insLLH(:,i-1),omega_b_ib,ecc,Ro,insVel(:,i-1),dtIMU
        // output= insAtt(:,i), Cb2n+,Cb2n-,Omega_n_en,Omega_n_ie,R_N,R_E,omega_n_in,omega_n_ie
        //------------------------------------------------------------------------------
        CoreNav::Matrix3 CnbMinus = CoreNav::eul_to_dcm(ins_att_[0],ins_att_[1],ins_att_[2]);
        CbnMinus=CnbMinus.transpose();
        omega_n_ie_ << INS::omega_ie*cos(ins_pos_[0]), 0.0, (-1.0)*INS::omega_ie*sin(ins_pos_[0]); // Checked
        Omega_b_ib_ = CoreNav::skew_symm( omega_b_ib_ );
        Omega_n_ie_ = CoreNav::skew_symm( omega_n_ie_ );
        // Radius of Curvature for North-South Motion (eq. 2.105)
        double R_N = INS::Ro*(1.0-pow(INS::ecc,2.0))/pow(1.0-pow(INS::ecc,2.0)*pow(sin(ins_pos_(0)),2.0),(3.0/2.0));
        // Radius of Curvature in East-West Direction (eq. 2.106)
        double R_E = INS::Ro/sqrt(1.0-pow(INS::ecc,2.0)*pow(sin(ins_pos_(0)),2.0));
        //rotation rate vector

        omega_n_en_ << ins_vel_(1)/(R_E+ins_pos_(2)),
        (-1.0)*ins_vel_(0)/(R_N+ins_pos_(2)),
        ((-1.0)*ins_vel_(1)*tan(ins_pos_(0)))/(R_E+ins_pos_(2));


        Omega_n_en_ = CoreNav::skew_symm( omega_n_en_ );
        //eq. 2.48
        omega_n_in_ = omega_n_en_ + omega_n_ie_;

        //integrate considering body-rate, Earth-rate, and craft-rate
        CoreNav::Matrix3 eye3=Eigen::Matrix3d::Identity();
        CoreNav::Matrix3 zeros3=Eigen::Matrix3d::Zero(3,3);
        CoreNav::Matrix3 CbnPlus;
        CbnPlus=CbnMinus*(eye3+Omega_b_ib_*dt_imu_)-(Omega_n_ie_+ Omega_n_en_)*CbnMinus*dt_imu_;
        ins_att_= CoreNav::dcm_to_eul( CbnPlus );

        //------------------------------------------------------------------------------
        // Velocity Update -------------------------------------------------------------
        // input= Cb2nMinus, Cb2nPlus, v_ib_b,insVel(:,i-1),insLLH(:,i-1),omega_ie,Ro,ecc,dtIMU
        // output=insVel(:,i)
        //------------------------------------------------------------------------------
        CoreNav::Vector3 V_n_ib;
        //specific-force transformation (eq. 5.48)
        V_n_ib=(1.0/2.0)*(CbnMinus+CbnPlus)*f_ib_b_*dt_imu_;
        grav_ = CoreNav::calc_gravity(ins_pos_(0),ins_pos_(2));
        CoreNav::Vector3 ins_velMinus(ins_vel_);
        ins_vel_= ins_vel_ + V_n_ib +  (grav_ - (Omega_n_en_+2.0*(Omega_n_ie_))*ins_vel_)*dt_imu_;
        //ROS_INFO("ins_vel = [%f,%f,%f]",ins_vel_[0],ins_vel_[1],ins_vel_[2]);
        //------------------------------------------------------------------------------
        // Position Update -------------------------------------------------------------
        // input= insLLH(:,i-1),insVel(:,i),insVel(:,i-1),Ro,ecc,dtIMU
        // output= insLLH(:,i)
        //------------------------------------------------------------------------------
        double heightPlus=ins_pos_(2)-(dt_imu_/2.0)*(ins_velMinus(2)+ins_vel_(2));
        double latPlus = ins_pos_(0)+(dt_imu_/2.0)*(ins_velMinus(0)/(R_N+ins_pos_(2))+ins_vel_(0)/(R_N+heightPlus));
        double R_EPlus=INS::Ro/sqrt(1.0-pow(INS::ecc,2.0)*pow(sin(latPlus),2.0));
        double lonPlus=ins_pos_(1)+(dt_imu_/2.0)*(ins_velMinus(1)/((R_E+ins_pos_(2))*cos(ins_pos_(0)))+ins_vel_(1)/((R_EPlus+heightPlus)*cos(latPlus)));         //ins_pos_(1)

        ins_pos_ << latPlus,lonPlus,heightPlus;

        //------------------------------------------------------------------------------
        // State Transition Matrix -----------------------------------------------------
        // input= R_EPlus,R_N,insLLH(:,i),insVel(:,i),dtIMU,Cb2nPlus,omega_ie,omega_n_in,f_ib_b
        // output= STM
        //------------------------------------------------------------------------------
        STM_ = CoreNav::insErrorStateModel_LNF(R_EPlus, R_N, ins_pos_, ins_vel_, dt_imu_, CbnPlus, INS::omega_ie,omega_n_in_, f_ib_b_, INS::gravity);

        error_states_ = STM_*error_states_;
        //------------------------------------------------------------------------------
        // Q Matrix --------------------------------------------------------------------
        //------------------------------------------------------------------------------
        Q_ = CoreNav::calc_Q(R_N,R_EPlus,ins_pos_, dt_imu_, CbnPlus, f_ib_b_);
        //------------------------------------------------------------------------------
        // P Matrix --------------------------------------------------------------------
        //------------------------------------------------------------------------------
        // std::cout << "P" << '\n'<< P << '\n';
        // std::cout << "P_" << '\n'<< P_ << '\n';
        P_=STM_*P_*STM_.transpose()+ Q_;
        // std::cout << "P_2" << '\n'<< P_ << '\n';
// ros::shutdown();

        //------------------------------------------------------------------------------
        // Measurement Matrix ----------------------------------------------------------
        //------------------------------------------------------------------------------
        CoreNav::Matrix3 Cn2bPlus=CbnPlus.transpose();
        CoreNav::Vector3 omega_b_ie=Cn2bPlus*omega_n_ie_;
        CoreNav::Vector3 omega_b_ei=-1.0*omega_b_ie;
        CoreNav::Vector3 omega_b_eb=omega_b_ei+omega_b_ib_;

        // TODO: WE NEED FIXED VALUES FOR STM Q AND P WHEN WE RUN THE GP

        if (has_odo_)
        {
                CoreNav::Matrix3 ins_vel_ss; //ins_vel_ skew symmetric
                ins_vel_ss = CoreNav::skew_symm(ins_vel_);
                CoreNav::Vector3 Val_H21(0,cos(ins_att_(0)),sin(ins_att_(0)));

                H11_ += eye3.row(0)*Cn2bPlus*ins_vel_ss*dt_imu_;
                H12_ += eye3.row(0)*Cn2bPlus*dt_imu_;
                H21_ += sin(ins_att_(1))*Val_H21.transpose()*Cn2bPlus*dt_imu_;
                H31_ += eye3.row(1)*Cn2bPlus*ins_vel_ss*dt_imu_;
                H32_ += eye3.row(1)*Cn2bPlus*dt_imu_;
                H41_ += eye3.row(2)*Cn2bPlus*ins_vel_ss*dt_imu_;
                H42_ += eye3.row(2)*Cn2bPlus*dt_imu_;

                CoreNav::Matrix3 Omega_b_eb;
                Omega_b_eb << 0.0, -1.0*omega_b_eb(2), omega_b_eb(1),
                omega_b_eb(2), 0.0, -1.0*omega_b_eb(0),
                -1.0*omega_b_eb(1), omega_b_eb(0), 0.0;

                // Measurement Innovation -- integration part for INS -- eq 16.42
                double tmp1 = eye3.row(0)*(Cn2bPlus*ins_vel_ + (CoreNav::skew_symm(omega_b_eb)*(-0.272*(eye3.col(0)))));
                double tmp2 = eye3.row(1)*(Cn2bPlus*ins_vel_ + (CoreNav::skew_symm(omega_b_eb)*(-0.272*(eye3.col(0)))));
                double tmp3 = eye3.row(2)*(Cn2bPlus*ins_vel_ + (CoreNav::skew_symm(omega_b_eb)*(-0.272*(eye3.col(0)))));

                z11_ += tmp1*dt_imu_;
                z21_ += cos(ins_att_(1))*dt_imu_;
                z31_ += tmp2* dt_imu_;
                z41_ += tmp3 * dt_imu_;

        }


        CoreNav::NonHolonomic(ins_vel_, ins_att_, ins_pos_, error_states_, P_, omega_b_ib_);
 //std::cout << "rearVel_" << '\n'<< rearVel_ << '\n';
        if ( std::abs(rearVel_) < 0.005) // TODO: Revisit here
        {
                CoreNav::zupt(ins_vel_, ins_att_, ins_pos_, error_states_, P_);
                CoreNav::zaru(ins_vel_, ins_att_, ins_pos_, error_states_, P_, omega_b_ib_);
        }
        ba_(0)=error_states_(9);
        ba_(1)=error_states_(10);
        ba_(2)=error_states_(11);
        bg_(0)=error_states_(12);
        bg_(1)=error_states_(13);
        bg_(2)=error_states_(14);

        ins_enu_ << CoreNav::llh_to_enu(ins_pos_[0],ins_pos_[1],ins_pos_[2]);
        ins_cn_<<ins_att_,ins_vel_,ins_enu_;
        ins_xyz_ << CoreNav::llh_to_xyz(ins_pos_[0],ins_pos_[1],ins_pos_[2]);
        ins_pos_llh_ << ins_pos_[0]*180.0/INS::PI,ins_pos_[1]*180.0/INS::PI, ins_pos_[2];

        PublishStates(ins_att_, attitude_pub_);
        PublishStates(ins_vel_, velocity_pub_);
        PublishStates(ins_pos_, position_pub_);

        PublishStates(ins_enu_, enu_pub_);
        PublishStates(ins_xyz_, ins_xyz_pub_);
        PublishStates(ins_pos_llh_, pos_llh_pub_);

        PublishStatesCN(ins_cn_, cn_pub_);

        return;
}

void CoreNav::Update(const CoreNav::Vector13& odo,const CoreNav::Vector4& joint)
{
        odomUptCount=odomUptCount+1;
        rearVel_ = (joint[3]-joint[2])*INS::wheel_radius/2.0;
        headRate_ = ((-joint[0]-joint[2])/2.0-(joint[3]+joint[1])/2.0)*INS::wheel_radius/(0.5); //TUNE 0.5


        velFrontLeft_=-joint[0]*INS::wheel_radius;
        velFrontRight_=joint[1]*INS::wheel_radius;
        velBackLeft_=-joint[2]*INS::wheel_radius;
        velBackRight_=joint[3]*INS::wheel_radius;

        dt_odo_ = odo_stamp_curr_ - odo_stamp_prev_;
        CoreNav::Matrix3 Cn2bUnc=CoreNav::eul_to_dcm(ins_att_(0),ins_att_(1),ins_att_(2));
        H11_ =-H11_/dt_odo_;
        H12_ =-H12_/dt_odo_;
        H21_ = H21_*(ins_att_(2)-psiEst)/(dt_odo_*dt_odo_); // TODO Check here
        H31_ =-H31_/dt_odo_;
        H32_ =-H32_/dt_odo_;
        H24_ =-(cos(ins_att_(1))*eye3.row(2)*Cn2bUnc.transpose())/dt_odo_;
        H41_ =-H41_/dt_odo_;
        H42_ =-H42_/dt_odo_;

        double z1_odom=rearVel_*(1-s_or_);
        double z2_odom=headRate_*(1-s_or_)-((z11_/dt_odo_)/T_r_)*s_delta_or_;
        double z1_ins=z11_;
        double z2_ins=(ins_att_(2)-init_yaw)*z21_; // TODO Check here
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
        H_.row(1)<<H21_.transpose(), zeros3.row(0), zeros3.row(0), H24_.transpose(), zeros3.row(0);
        H_.row(2)<<H31_.transpose(), H32_.transpose(), zeros3.row(0), zeros3.row(0),zeros3.row(0);
        H_.row(3)<<H41_.transpose(), H42_.transpose(), zeros3.row(0), zeros3.row(0),zeros3.row(0);

        K_ << P_*H_.transpose()*(H_*P_*H_.transpose()+R_).inverse();

        error_states_ = error_states_+K_*(Z_-H_*error_states_);

        CoreNav::Vector3 att_error_states(error_states_(0),error_states_(1),error_states_(2));
        CoreNav::Matrix3 error_states_ss;
        error_states_ss=CoreNav::skew_symm(att_error_states);
        CoreNav::Matrix3 insAttEst;
        insAttEst<<(eye3-error_states_ss)*Cn2bUnc.transpose();

        double phiEst = atan2(insAttEst(2,1),insAttEst(2,2));
        double thetaEst = asin(-insAttEst(2,0));
        double psiEst = atan2(insAttEst(1,0),insAttEst(0,0));

        ins_att_ << phiEst, thetaEst, psiEst;
        ins_vel_ << ins_vel_(0)-error_states_(3), ins_vel_(1)-error_states_(4), ins_vel_(2)-error_states_(5);
        ins_pos_ << ins_pos_(0)-error_states_(6), ins_pos_(1)-error_states_(7), ins_pos_(2)-error_states_(8);



        init_yaw=ins_att_(2);

        error_states_.segment(0,9)<<Eigen::VectorXd::Zero(9);

        P_=(Eigen::MatrixXd::Identity(15,15) - K_*H_) * P_ * ( Eigen::MatrixXd::Identity(15,15) - K_ * H_ ).transpose() + K_ * R_ * K_.transpose();


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

if (slip !=0.0 && slip !=-1.0 && slip !=1.0)
{
  if (flag)
  {
    ROS_WARN_ONCE("Driving Started at %.2f sec", odomUptCount/10.0);
    saveCountOdom= odomUptCount;
    startRecording=saveCountOdom+10;
    stopRecording=startRecording+150;
    ROS_INFO_ONCE("Recording will be started at %.2f sec", startRecording/10.0);
    ROS_INFO_ONCE("Recording will be stopped at %.2f sec", stopRecording/10.0);
    // ROS_ERROR("XYerror1 %.12f meters", xy_errSlip);

    flag=false;
  }

  if (odomUptCount > startRecording && odomUptCount < stopRecording && !gp_flag)
  {

    if (std::isnan(slip)) {
      ROS_ERROR_ONCE("SLIP IS NaN - Restart required");
    }
    ROS_WARN_THROTTLE(10,"Recording GP Input between %.2f sec and %.2f sec",startRecording/10.0,stopRecording/10.0);
    slip_msg.slip_array.push_back(slip);
    slip_msg.time_array.push_back(odomUptCount);


  }
  if (odomUptCount==stopRecording)
  {
    savePos=ins_pos_;
    P_pred=P_;

    if(!gp_flag)
    {
            ROS_WARN("Publishing GP Input at %.2f sec", stopRecording/10.0);
            gp_flag=true;
            gp_pub.publish(slip_msg);
            slip_msg.slip_array.clear();
            slip_msg.time_array.clear();
    }
    // std::cout << "savePos" << '\n'<< savePos << '\n';
    // ROS_WARN("At %.2f,  Saved Position Lat= %.6f, Lon=%.6f, Height=%.6f",odomUptCount,savePos(0),savePos(1),savePos(2));

    if(new_gp_data_arrived_)
    {
        for(int slip_i=0; slip_i<25*gp_data_.mean.size(); slip_i++)
        {
            P_pred = STM_*P_pred*STM_.transpose() + Q_; //TODO: Should I create a new variable like STM_fixed and Q_fixed? I think we only use the latest one while in this section it doesn't change.
            if ((slip_i % 25) ==0)
            {
            double chi0_slip=gp_data_.mean.at(i);
            double chi1_slip=gp_data_.mean.at(i)+gp_data_.sigma.at(i);
            double chi2_slip=gp_data_.mean.at(i)-gp_data_.sigma.at(i);

            double chi0_odo=0.7/(1.0-chi0_slip); //0.6536//TODO: Take the average velocity instead of hard coded 0.6536
            double chi1_odo=0.7/(1.0-chi1_slip);
            double chi2_odo=0.7/(1.0-chi2_slip);

            double chi_UT_est=(chi0_odo+chi1_odo+chi2_odo)/3.0;
            // TODO: The scaling constant is 5.0, but why?
            double chi_UT_est_cov=((chi0_odo-chi_UT_est)*(chi0_odo-chi_UT_est)+(chi1_odo-chi_UT_est)*(chi1_odo-chi_UT_est)+(chi2_odo-chi_UT_est)*(chi2_odo-chi_UT_est))/3.0;


             R_IP_2<<  std::max(0.03*0.03,chi_UT_est_cov*chi_UT_est_cov), 0.0,0.0,0.0,
                    0, std::max(0.03*0.03,chi_UT_est_cov*chi_UT_est_cov),  0,0,
                    0, 0, std::max(0.05*0.05,chi_UT_est_cov*chi_UT_est_cov),0,
                    0,0,0,0.05*0.05;


          R_IP=R_IP_1*R_IP_2*R_IP_1.transpose();

            K_pred=P_pred*H_.transpose()*(H_*P_pred*H_.transpose() +R_IP).inverse();
            P_pred=(Eigen::MatrixXd::Identity(15,15) - K_pred*H_)*P_pred*(Eigen::MatrixXd::Identity(15,15)-K_pred*H_).transpose()  + K_pred*R_IP*K_pred.transpose();
            // std::cout << "i:" << i << '\n';
            i++;
            }
            // std::cout << "chi_UT_est:"<< chi_UT_est << '\n';
            // std::cout << "chi_UT_est_cov:"<< chi_UT_est_cov << '\n';
            // std::cout << "ppred6:" << 3.0*sqrt(std::abs(P_pred(6,6))) << '\n';
            // std::cout << "ppred7:" << 3.0*sqrt(std::abs(P_pred(7,7))) << '\n';
            // std::cout << "ppred8:" << 3.0*sqrt(std::abs(P_pred(8,8))) << '\n';

            ins_enu_slip << CoreNav::llh_to_enu(savePos[0],savePos[1],savePos[2]);
            ins_enu_slip_3p << CoreNav::llh_to_enu(savePos[0]-3.0*sqrt(std::abs(P_pred(6,6))),savePos[1]-3.0*sqrt(std::abs(P_pred(7,7))), savePos[2]-3.0*sqrt(std::abs(P_pred(8,8))));
            ins_enu_slip3p << CoreNav::llh_to_enu(savePos[0]+3.0*sqrt(std::abs(P_pred(6,6))),savePos[1]+3.0*sqrt(std::abs(P_pred(7,7))), savePos[2]+3.0*sqrt(std::abs(P_pred(8,8))));



            xy_errSlip = sqrt((ins_enu_slip3p(0)-ins_enu_slip(0))*(ins_enu_slip3p(0)-ins_enu_slip(0)) + (ins_enu_slip3p(1)-ins_enu_slip(1))*(ins_enu_slip3p(1)-ins_enu_slip(1)));
            ROS_ERROR("XYerror %.6f meters", xy_errSlip);
            // std::cout << "error" << '\n'<< xy_errSlip << '\n';
            if (xy_errSlip > 1.00) { //TODO: the error 3.00 meters is too much.

              ROS_ERROR_ONCE("Stop Command Required, error is more than %.2f meters", xy_errSlip);
              ROS_ERROR_ONCE("Stop command should be set at %u seconds after %.2f sec driving",i/10,odomUptCount/10.0);
              if (gp_arrived_time_ + i/10.0 - ros::Time::now().toSec()<0.0) {
              // if (gp_arrived_time_ + i/10.0 <0.0) {
                stop_cmd_msg_.data = 0.5;
              } else {
                stop_cmd_msg_.data = gp_arrived_time_ + i/10.0 - ros::Time::now().toSec();
                // stop_cmd_msg_.data = gp_arrived_time_ + i/10.0 ;

              }
              stop_cmd_pub_.publish(stop_cmd_msg_);
              ROS_ERROR_ONCE("delta_time = %.3f", stop_cmd_msg_.data);

              break;
            }
        }
        // std::cout << "error2" << '\n'<< xy_errSlip << '\n';
        // std::cout << "slip_i2" << '\n'<< slip_i << '\n';
        new_gp_data_arrived_ = false; // Set the flag back to false, so that this does not happen again until new data comes in on the subscriber callback and sets this flag back to true
        i=0.0;
        slip_i=0.0;
    }

    startRecording=stopRecording;
    stopRecording=startRecording+150;
    ROS_INFO_ONCE("Start Recording 2 at %.2f", startRecording);
    ROS_INFO_ONCE("Stop Recording 2 set at %.2f", stopRecording);

  }
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
        return;
}
void CoreNav::GPCallBack(const core_nav::GP_Output::ConstPtr& gp_data_in_)
{
    this->gp_data_.mean = gp_data_in_->mean;
    this->gp_data_.sigma = gp_data_in_->sigma;
    new_gp_data_arrived_ = true;
    ROS_INFO("New GP data is available for %.2f seconds", gp_data_.mean.size()/10.0);
    gp_arrived_time_ = ros::Time::now().toSec();

}

CoreNav::Vector6 CoreNav::getImuData(const ImuData& imu_dataAdis_)
{
        CoreNav::Vector6 imuVec((Vector(6) << imu_dataAdis_.linear_acceleration.x,
                                 imu_dataAdis_.linear_acceleration.y,
                                 imu_dataAdis_.linear_acceleration.z,
                                 imu_dataAdis_.angular_velocity.x,
                                 imu_dataAdis_.angular_velocity.y,
                                 imu_dataAdis_.angular_velocity.z).finished());
        return imuVec;
}

CoreNav::Vector4 CoreNav::getJointData(const JointData& joint_data_)
{
        CoreNav::Vector4 jointVec((Vector(4) << joint_data_.velocity[0],
                                   joint_data_.velocity[1],
                                   joint_data_.velocity[2],
                                   joint_data_.velocity[3] ).finished());
        return jointVec;
}
CoreNav::Vector CoreNav::getCmdData(const CmdData& cmd_data_)
{
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

CoreNav::Vector2 CoreNav::getEncoderLeftData(const EncoderLeftData& encoderLeft_data_)
{
        CoreNav::Vector2 encoderLeftVec((Vector(2) << encoderLeft_data_.encoder_counter_absolute[0],
                                   encoderLeft_data_.encoder_counter_absolute[1]).finished());
        return encoderLeftVec;
}

CoreNav::Vector2 CoreNav::getEncoderRightData(const EncoderRightData& encoderRight_data_)
{
        CoreNav::Vector2 encoderRightVec((Vector(2) << encoderRight_data_.encoder_counter_absolute[0],
                                   encoderRight_data_.encoder_counter_absolute[1]).finished());
        return encoderRightVec;
}

CoreNav::Vector13 CoreNav::getOdoData(const OdoData& odo_data_)
{

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



// Publish estimated states in global frame
void CoreNav::PublishStates(const CoreNav::Vector3& states,
                            const ros::Publisher& pub){
        // // Check for subscribers before doing any work.
        if(pub.getNumSubscribers() == 0)
                return;

        geometry_msgs::PointStamped msg;

        msg.point.x = states(0);
        msg.point.y = states(1);
        msg.point.z = states(2);
        msg.header.frame_id = frame_id_fixed_;
        msg.header.stamp = stamp_;

        pub.publish(msg);
}
void CoreNav::PublishStatesSlip(const CoreNav::Vector3& slip_states,
                            const ros::Publisher& slip_pub){
        // // Check for subscribers before doing any work.
        if(slip_pub.getNumSubscribers() == 0)
                return;

        geometry_msgs::PointStamped msg;

        msg.point.x = slip_states(0); //slip
        msg.point.y = slip_states(1); //rearWheelLinearVelocity
        msg.point.z = slip_states(2); //INSestimatedLinearVelocityX
        msg.header.frame_id = frame_id_fixed_;
        msg.header.stamp = stamp_;

        slip_pub.publish(msg);
}

void CoreNav::PublishStatesCN(const CoreNav::Vector9& cn_states,
                              const ros::Publisher& cn_pub_){
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
        CNmsg.header.stamp = stamp_;

        cn_pub_.publish(CNmsg);
}

CoreNav::Vector3 CoreNav::calc_gravity(const double latitude, const double height)
{
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

CoreNav::Matrix3 CoreNav::skew_symm(const CoreNav::Vector3 vec)
{
        CoreNav::Matrix3 ss;
        ss << 0.0, -1.0*vec(2), vec(1),
        vec(2), 0.0, -1.0*vec(0),
        -1.0*vec(1), vec(0), 0.0;

        return ss;
}

CoreNav::Matrix3 CoreNav::eul_to_dcm(double phi, double theta, double psi)
{
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

CoreNav::Vector3 CoreNav::dcm_to_eul(CoreNav::Matrix3 dcm)
{
        CoreNav::Vector3 eul;
        eul << std::atan2(dcm(2,1), dcm(2,2)), std::asin(-1*dcm(2,0)), std::atan2(dcm(1,0), dcm(0,0));
        return eul;
}

// TODO
CoreNav::Vector3 CoreNav::llh_to_enu(double lat, double lon, double height)
{


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
CoreNav::Vector3 CoreNav::llh_to_xyz(double lat, double lon, double height)
{


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

// NonHolonomic Update
void CoreNav::NonHolonomic(const CoreNav::Vector3 vel, const CoreNav::Vector3 att, const CoreNav::Vector3 llh, CoreNav::Vector15 error_states, Eigen::MatrixXd P, CoreNav::Vector3 omega_b_ib)
{

        CoreNav::Matrix3 Cnb = CoreNav::eul_to_dcm(att[0],att[1],att[2]);

        z_holo.row(0) = -eye3.row(1)*(Cnb*vel-CoreNav::skew_symm(omega_b_ib)*(-0.272*(eye3.col(0))));
        z_holo.row(1) = -eye3.row(2)*(Cnb*vel-CoreNav::skew_symm(omega_b_ib)*(-0.272*(eye3.col(0))));

        H_holo.row(0)<<zeros3.row(0), -eye3.row(1)*Cnb, zeros3.row(0), zeros3.row(0), zeros3.row(0);
        H_holo.row(1)<<zeros3.row(0), -eye3.row(2)*Cnb, zeros3.row(0), zeros3.row(0), zeros3.row(0);

        K_holo = P * H_holo.transpose() * (H_holo * P * H_holo.transpose() + R_holo).inverse();

        error_states_ = error_states + K_holo* (z_holo  - H_holo * error_states);

        ins_att_ = CoreNav::dcm_to_eul((Eigen::MatrixXd::Identity(3,3)- CoreNav::skew_symm(error_states_.segment(0,3)))*Cnb.transpose());
        ins_vel_ = vel - error_states_.segment(3,3);
        ins_pos_ = llh - error_states_.segment(6,3);

        error_states_.segment(0,9)<<Eigen::VectorXd::Zero(9);

        P_=(Eigen::MatrixXd::Identity(15,15) - K_holo * H_holo) * P * ( Eigen::MatrixXd::Identity(15,15) - K_holo * H_holo ).transpose() + K_holo * R_holo * K_holo.transpose();

        return;
}


// Zero vel. update
void CoreNav::zupt(const CoreNav::Vector3 vel, const CoreNav::Vector3 att, const CoreNav::Vector3 llh, CoreNav::Vector15 error_states, Eigen::MatrixXd P)
{
        CoreNav::Matrix3 Cnb = CoreNav::eul_to_dcm(att[0],att[1],att[2]);

        CoreNav::Vector3 z_zupt;
        z_zupt = -vel;

        K_zupt = P * H_zupt.transpose() * (H_zupt * P * H_zupt.transpose() + R_zupt).inverse();

        error_states_ = error_states + K_zupt * (z_zupt  - H_zupt * error_states);

        ins_att_ = CoreNav::dcm_to_eul((Eigen::MatrixXd::Identity(3,3)- CoreNav::skew_symm(error_states_.segment(0,3)))*Cnb.transpose());
        ins_vel_ = vel - error_states_.segment(3,3);
        ins_pos_ = llh - error_states_.segment(6,3);

        error_states_.segment(0,9)<<Eigen::VectorXd::Zero(9);


        P_=(Eigen::MatrixXd::Identity(15,15) - K_zupt * H_zupt) * P * ( Eigen::MatrixXd::Identity(15,15) - K_zupt * H_zupt ).transpose() + K_zupt * R_zupt * K_zupt.transpose();

        return;
}

// Zero ang. update
void CoreNav::zaru(const CoreNav::Vector3 vel, const CoreNav::Vector3 att, const CoreNav::Vector3 llh, CoreNav::Vector15 error_states, Eigen::MatrixXd P, const CoreNav::Vector3 omega_b_ib)
{

        CoreNav::Matrix3 Cnb = CoreNav::eul_to_dcm(att[0],att[1],att[2]);

        CoreNav::Vector3 z_zaru;
        z_zaru = -omega_b_ib.transpose();

        K_zaru = P * H_zaru.transpose() * (H_zaru * P * H_zaru.transpose() + R_zaru).inverse();

        error_states_ = error_states + K_zaru * (z_zaru  - H_zaru * error_states);

        ins_att_ = CoreNav::dcm_to_eul((Eigen::MatrixXd::Identity(3,3)- CoreNav::skew_symm(error_states_.segment(0,3)))*Cnb.transpose());
        ins_vel_ = vel - error_states_.segment(3,3);
        ins_pos_ = llh - error_states_.segment(6,3);

        error_states_.segment(0,9)<<Eigen::VectorXd::Zero(9);


        P_=(Eigen::MatrixXd::Identity(15,15) - K_zaru * H_zaru) * P * ( Eigen::MatrixXd::Identity(15,15) - K_zaru * H_zaru ).transpose() + K_zaru * R_zaru * K_zaru.transpose();
        return;
}


CoreNav::Matrix CoreNav::insErrorStateModel_LNF(double R_EPlus, double R_N, CoreNav::Vector3 insLLH, CoreNav::Vector3 insVel, double dt, CoreNav::Matrix3 CbnPlus, double omega_ie,CoreNav::Vector3 omega_n_in,CoreNav::Vector3 f_ib_b,double gravity)
{
        double geoLat= atan2(INS::t_const*sin(insLLH(0)*180.0/INS::PI), cos(insLLH(0)*180.0/INS::PI));
        double rGeoCent  = pow(( pow(INS::Ro,2.0) /( 1.0 + (1.0/(pow(( 1.0 - INS::flat ),2.0)) - 1.0)*pow(sin(geoLat),2.0))),(1.0/2.0));
        double g0 = 9.780318*( 1.0 + (5.3024e-3)*pow(sin(insLLH(0)),2.0) - (5.9e-6)*pow(sin(2*insLLH(0)),2.0) );

        CoreNav::Matrix3 F11;
        F11= CoreNav::skew_symm(-omega_n_in);

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

// TODO:: This needs to read in the values specifed in parameters.yaml
CoreNav::Matrix CoreNav::calc_Q(double R_N, double R_E, CoreNav::Vector3 insLLH, double dt, CoreNav::Matrix3 CbnPlus,CoreNav::Vector3 f_ib_b)
{
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
