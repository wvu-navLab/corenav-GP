/**
 * @file   insUtils.cpp
 * @brief  Tools required to process inertial data
 * @authors Cagri, Ryan
 */

#include <core_navigation/InsUtils.h>

namespace INS {

Vector3 calc_gravity(const double latitude, const double height)
{
        double e2=pow(ecc,2.0);
        double den=1.0-e2*pow(sin(latitude),2.0);
        double Rm=Ro*(1.0-e2)/pow(den,(3.0/2.0));
        double Rp=Ro/pow(den,(1.0/2.0));
        double Top=Ro*pow((pow((1.0-e2),2.0)+pow((sin(latitude)),2.0)+pow((cos(latitude)),2.0)),(1.0/2.0));
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
        Vector3 grav(0.0,0.0,gravity);
        return grav;
}

Matrix3 skew_symm(const Vector3 vec)
{
        Matrix3 ss;
        ss << 0.0, -1.0*vec(2), vec(1),
        vec(2), 0.0, -1.0*vec(0);
        -1.0*vec(1), vec(0), 0.0;

        return ss;
}

Matrix3 eul_to_dcm(double phi, double theta, double psi)
{
        Matrix3 CbnMinus; //body2nav
        double cpsi = cos(psi); double spsi = sin(psi);
        double cthe = cos(theta); double sthe = sin(theta);
        double cphi = cos(phi); double sphi = sin(phi);

        Matrix3 c1; //y
        c1.row(0)<<cpsi, spsi, 0.0; //c11
        c1.row(1)<<(-1.0)*spsi, cpsi,0.0; //c12
        c1.row(2)<<0.0, 0.0,1.0;  //c13

        Matrix3 c2; //p
        c2.row(0)<<cthe,0.0,(-1.0)*sthe; //c21
        c2.row(1)<<0.0,1.0,0.0;   //c22
        c2.row(2)<<sthe,0.0,cthe; //c23

        Matrix3 c3; //r
        c3.row(0)<<1.0,0.0,0.0;   //c31
        c3.row(1)<<0.0,cphi,sphi; //c32
        c3.row(2)<<0.0,(-1.0)*sphi,cphi; //c33

        Matrix3 DCMnb= (c3*c2)*c1; //CnbMinus;nav2body
        CbnMinus=DCMnb.transpose(); //body2nav
        return CbnMinus;
}

Matrix insErrorStateModel_LNF(double R_EPlus, double R_N, Vector3 insLLHPlus, Vector3 insVelPlus, double dt, Matrix3 CbnPlus, double omega_ie,Vector3 omega_n_in,Vector3 f_ib_b,double gravity)
{

        double geoLat= atan2(t_const*sin(insLLHPlus(0)*180.0/INS::PI), cos(insLLHPlus(0)*180.0/INS::PI));
        double rGeoCent  = pow(( pow(Ro,2.0) /( 1.0 + (1.0/(pow(( 1.0 - flat ),2.0)) - 1.0)*pow(sin(geoLat),2.0))),(1.0/2.0));
        double g0 = 9.780318*( 1.0 + (5.3024e-3)*pow(sin(insLLHPlus(0)),2.0) - (5.9e-6)*pow(sin(2*insLLHPlus(0)),2.0) );

        Matrix3 F12; //Checked
        F12.row(0)<<0.0, -1.0/(R_EPlus+insLLHPlus(2)), 0.0; //height
        F12.row(1)<<1.0/(R_N+insLLHPlus(2)), 0.0, 0.0; //height
        F12.row(2)<<0.0, tan(insLLHPlus(0))/(R_EPlus+insLLHPlus(2)), 0.0; //lat/height

        Matrix3 F13; //Checked
        F13.row(0)<<omega_ie*sin(insLLHPlus(0)), 0.0, insVelPlus(1)/pow((R_EPlus+insLLHPlus(2)),2.0);
        F13.row(1)<<0.0, 0.0, -insVelPlus(0)/pow((R_N+insLLHPlus(2)),2.0);
        F13.row(2)<<omega_ie*cos(insLLHPlus(0))+insVelPlus(1)/(R_EPlus+insLLHPlus(2))*pow(cos(insLLHPlus(0)),2.0), 0.0, -1.0*insVelPlus(1)*tan(insLLHPlus(0))/pow((R_EPlus+insLLHPlus(2)),2.0);

        Matrix3 F22; //Checked
        F22.row(0)<<insVelPlus(2)/(R_N+insLLHPlus(2)), -(2.0)*insVelPlus(1)*tan(insLLHPlus(1))/(R_EPlus+insLLHPlus(2))-(2.0)*omega_ie*sin(insLLHPlus(0)), insVelPlus(0)/(R_N+insLLHPlus(2));
        F22.row(1)<<(insVelPlus(1)*tan(insLLHPlus(0))/(R_EPlus+insLLHPlus(2)))+(2.0)*omega_ie*sin(insLLHPlus(0)), (insVelPlus(0)*tan(insLLHPlus(0))+insVelPlus(2))/(R_EPlus+insLLHPlus(2)), (insVelPlus(1)/(R_EPlus+insLLHPlus(2)))+(2.0)*omega_ie*cos(insLLHPlus(0));
        F22.row(2)<<(-2.0)*insVelPlus(0)/(R_N+insLLHPlus(2)), (-2.0)*(insVelPlus(1)/(R_EPlus+insLLHPlus(2)))-(2.0)*omega_ie*cos(insLLHPlus(0)), 0.0;

        Matrix3 F23; //Checked
        F23.row(0)<<(-1.0*pow(insVelPlus(1),2.0)/(pow(cos(insLLHPlus(0)),2.0)*(R_EPlus+insLLHPlus(2))))-(2.0*insVelPlus(1)*omega_ie*cos(insLLHPlus(0))), 0.0, (((pow(insVelPlus(1),2.0))*(tan(insLLHPlus(0))))/(pow((R_EPlus+insLLHPlus(2)),2.0)))-((insVelPlus(0)*insVelPlus(2))/pow((R_N+insLLHPlus(2)),2.0));
        F23.row(1)<<((insVelPlus(0)*insVelPlus(1))/(pow(cos(insLLHPlus(0)),2.0)*(R_EPlus+insLLHPlus(2))))+(2.0*insVelPlus(0)*omega_ie*cos(insLLHPlus(0)))-(2.0*insVelPlus(2)*omega_ie*sin(insLLHPlus(0))), 0.0, -1.0*(insVelPlus(0)*insVelPlus(1)*tan(insLLHPlus(0))+insVelPlus(1)*insVelPlus(2))/(pow((R_EPlus+insLLHPlus(2)),2.0));
        F23.row(2)<<(2.0)*insVelPlus(1)*omega_ie*sin(insLLHPlus(0)), 0.0, (((pow(insVelPlus(1),2.0))/pow((R_EPlus+insLLHPlus(2)),2.0))+((pow(insVelPlus(0),2.0))/(R_N+insLLHPlus(2)))-((2*g0)/(rGeoCent)));

        Matrix3 F32; //Checked
        F32.row(0)<<(1.0)/(R_N+insLLHPlus(2)), 0.0, 0.0;
        F32.row(1)<<0.0, (1.0)/((R_EPlus+insLLHPlus(2))*cos(insLLHPlus(0))), 0.0;
        F32.row(2)<<0.0, 0.0, -1.0;

        Matrix3 F33; // Checked
        F33.row(0)<<0.0, 0.0, -insVelPlus(0)/pow((R_N+insLLHPlus(2)),2.0);
        F33.row(1)<<(insVelPlus(1)*sin(insLLHPlus(0)))/((R_EPlus+insLLHPlus(2))*pow(cos(insLLHPlus(0)),2.0)), 0.0, (insVelPlus(1)*(-1.0))/(pow((R_EPlus+insLLHPlus(2)),2.0)*cos(insLLHPlus(0)));
        F33.row(2)<<0.0, 0.0, 0.0;

        Matrix3 F11; //skewsymm((-1.0)*omega_n_in); %Omega_n_in %Eq:14.64
        F11.row(0)<<0.0, omega_n_in(2), (-1.0)*omega_n_in(1);
        F11.row(1)<<(-1.0)*omega_n_in(2), 0.0, omega_n_in(0);
        F11.row(2)<<omega_n_in(1), (-1.0)*omega_n_in(0), 0.0;

        Vector3 F21_ = (-1.0)*CbnPlus*(f_ib_b); //
        Matrix3 F21; //skewsymm(-CbnPlus*(delv)/dt); %Eq:14.67
        F21.row(0)<<0.0, (-1.0)*F21_(2), F21_(1);
        F21.row(1)<<F21_(2), 0.0, (-1.0)*F21_(0);
        F21.row(2)<<(-1.0)*F21_(1), F21_(0), 0.0;

        Matrix3d PHI11 = Matrix3d::Identity(3,3)+F11*dt;
        Matrix3d PHI12 = F12*dt;
        Matrix3d PHI13 = F13*dt;
        Matrix3d PHI15 = CbnPlus*dt;
        Matrix3d PHI21 = F21*dt;
        Matrix3d PHI22 = Matrix3d::Identity()+F22*dt;
        Matrix3d PHI23 = F23*dt;
        Matrix3d PHI24 = CbnPlus*dt;
        Matrix3d PHI32 = F32*dt;
        Matrix3d PHI33 = Matrix3d::Identity()+F33*dt;

//%Eq:14.72
        Matrix STM(15,15);

        STM<<PHI11, PHI12, PHI13, Matrix3d::Zero(3,3),PHI15,
        PHI21, PHI22, PHI23, PHI24, Matrix3d::Zero(3,3),
        Matrix3d::Zero(3,3), PHI32, PHI33, Matrix3d::Zero(3,3), Matrix3d::Zero(3,3),Matrix3d::Zero(3,3),
        Matrix3d::Zero(3,3),Matrix3d::Zero(3,3),Matrix3d::Identity(3,3),Matrix3d::Zero(3,3),Matrix3d::Zero(3,3),
        Matrix3d::Zero(3,3),Matrix3d::Zero(3,3),Matrix3d::Zero(3,3),Matrix3d::Identity(3,3);

        return STM;
}


// TODO:: This needs to read in the values specifed in parameters.yaml
Matrix calc_Q(double R_N, double R_E, Vector3 insLLHPlus, double dt, Matrix3 CbnPlus,Vector3 f_ib_b)
{
        Vector3 F21_ = (-1.0)*CbnPlus*(f_ib_b); //
        Matrix3d T_rn_p;
        T_rn_p.row(0)<<1.0/(R_N+insLLHPlus(2)),0.0,0.0;
        T_rn_p.row(1)<<0.0,1.0/((R_E+insLLHPlus(2))*cos(insLLHPlus(0))),0.0;
        T_rn_p.row(2)<<0.0,0.0,-1.0;

        Matrix3d F21; //skewsymm(-CbnPlus*(delv)/dt); Eq:14.67
        F21.row(0)<<0.0, (-1.0)*F21_(2), F21_(1);
        F21.row(1)<<F21_(2), 0.0, (-1.0)*F21_(0);
        F21.row(2)<<(-1.0)*F21_(1), F21_(0), 0.0;

        double sig_gyro_inRun = 1.6*INS::PI/180/3600; //rad/s
        double sig_ARW = .1*(INS::PI/180)*sqrt(3600)/3600;; //rad

        double sig_accel_inRun = (3.2e-6)*INS::gravity; // m/s
        double sig_VRW = 0.008*sqrt(3600)/3600; //m/s

//following 14.2.6 of Groves pp 592
        double Srg= pow(sig_ARW,2)*dt;
        double Sra= pow(sig_VRW,2)*dt;

        double Sbad=pow(sig_accel_inRun,2)/dt;
        double Sbgd=pow(sig_gyro_inRun,2)/dt;

        Matrix3d Q11 = (Srg*dt+(1.0/3.0)*Sbgd*pow(dt,3.0))*Matrix3d::Identity(3,3);
        Matrix3d Q21= ((1.0/2.0)*Srg*pow(dt,2)+(1.0/4.0)*Sbgd*pow(dt,4.0))*F21;
        Matrix3d Q12=Q21.transpose();
        Matrix3d Q31= ((1.0/3.0)*Srg*pow(dt,3.0)+(1.0/5.0)*Sbgd*pow(dt,5.0))*T_rn_p*F21;
        Matrix3d Q13=Q31.transpose();
        Matrix3d Q14=Matrix3d::Zero(3,3);
        Matrix3d Q15=(1.0/2.0)*Sbgd*pow(dt,2.0)*CbnPlus;
        Matrix3d Q22= (Sra*dt+(1.0/3.0)*Sbad*pow(dt,3.0))*Matrix3d::Identity(3,3)+((1.0/3.0)*Srg*pow(dt,2.0)+(1.0/5.0)*Sbgd*pow(dt,5.0))*F21*F21.transpose();
        Matrix3d Q32= ((1.0/2.0)*Sra*pow(dt,2.0)+(1.0/4.0)*Sbad*pow(dt,4))*T_rn_p+((1.0/4.0)*Srg*pow(dt,4.0)+(1.0/6.0)*Sbgd*pow(dt,6.0))*T_rn_p*F21*F21.transpose();
        Matrix3d Q23=Q32.transpose();
        Matrix3d Q24= (1.0/2.0)*Sbad*pow(dt,2.0)*CbnPlus;
        Matrix3d Q25= (1.0/3.0)*Sbgd*pow(dt,3.0)*F21*CbnPlus;
        Matrix3d Q33= ((1.0/3.0)*Sbad*pow(dt,3.0) + (1.0/5.0)*Sbad*pow(dt,5.0))*(T_rn_p*T_rn_p)+((1.0/5.0)*Srg*pow(dt,5.0)+(1.0/7.0)*Sbgd*pow(dt,7.0))*T_rn_p*F21*F21.transpose()*T_rn_p;
        Matrix3d Q34=(1.0/3.0)*Sbad*pow(dt,3.0)*T_rn_p*CbnPlus;
        Matrix3d Q35=(1.0/4.0)*Sbgd*pow(dt,4.0)*T_rn_p*F21*CbnPlus;
        Matrix3d Q41= Matrix3d::Zero(3,3);
        Matrix3d Q42= (1.0/2.0)*Sbad*pow(dt,2.0)*(CbnPlus.transpose());
        Matrix3d Q43=Q34.transpose();
        Matrix3d Q44=Sbad*dt*Matrix3d::Identity(3,3);
        Matrix3d Q45=Matrix3d::Zero(3,3);
        Matrix3d Q51= (1.0/2.0)*Sbgd*pow(dt,2.0)*(CbnPlus.transpose());
        Matrix3d Q52=(1.0/3.0)*Sbgd*pow(dt,3.0)*F21.transpose()*(CbnPlus.transpose());
        Matrix3d Q53=Q35.transpose();
        Matrix3d Q54=Matrix3d::Zero(3,3);
        Matrix3d Q55= Sbgd*dt*Matrix3d::Identity(3,3);

        Matrix Q(15,15);
        Q<<Q11,Q12,Q13,Q14,Q15,
        Q21,Q22,Q23,Q24,Q25,
        Q31,Q32,Q33,Q34,Q35,
        Q41,Q42,Q43,Q44,Q45,
        Q51,Q52,Q53,Q54,Q55;
        return Q;
}


}
