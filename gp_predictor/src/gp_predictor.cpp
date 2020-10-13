#include <ros/ros.h>
#include <gp_predictor/gp_predictor.h>

GpPredictor::GpPredictor(ros::NodeHandle &nh) :nh_(nh)
{


  // mobility_sub = nh_.subscribe("/state_machine/mobility_scout", 1, &GpPredictor::mobilityCallback, this);
  gp_sub_ =nh.subscribe("/core_nav/core_nav/gp_result",1, &GpPredictor::GPCallBack, this);
  clt_setStopping_ =nh_.serviceClient<core_nav::SetStopping>("/core_nav/core_nav/stopping_service");
  stop_cmd_pub_ = nh.advertise<std_msgs::Float64>("/core_nav/core_nav/stop_cmd", 1);

}


void GpPredictor::GPCallBack(const core_nav::GP_Output::ConstPtr& gp_data_in_){
    this->gp_data_.mean = gp_data_in_->mean;
    this->gp_data_.sigma = gp_data_in_->sigma;
    new_gp_data_arrived_ = true;
    ROS_INFO("New GP data is available for %.2f seconds", gp_data_.mean.size()/10.0);
    gp_arrived_time_ = ros::Time::now().toSec();

    core_nav::SetStopping srv_set_stopping;
    srv_set_stopping.request.stopping  = new_gp_data_arrived_;
    if (clt_setStopping_.call(srv_set_stopping))
    {

      savePos[0]=srv_set_stopping.response.PosData.x;
      savePos[1]=srv_set_stopping.response.PosData.y;
      savePos[2]=srv_set_stopping.response.PosData.z;

      // res.PosData=PosVec;
      for (int row=0; row<15; row++){
        for (int col=0; col<15; col++){
          P_pred(row,col)=srv_set_stopping.response.PvecData[row*15+col];
          Q_(row,col)=srv_set_stopping.response.QvecData[row*15+col];
          STM_(row,col)=srv_set_stopping.response.STMvecData[row*15+col];
        }
      }


      for (int row1=0; row1<4; row1++){
        for (int col1=0; col1<15; col1++){
          H_(row1,col1)=srv_set_stopping.response.HvecData[row1*4+col1];
        }
      }

    }
    else
    {
      ROS_ERROR(" Failed to call mobility service");
    }

    if(new_gp_data_arrived_) // This will happen after Gaussian Process publishes its results, check line 726. It initialized as false.
    {
      ROS_DEBUG("NEW GP DATA ARRIVED!");

      double cmd_stop_ = 0.0;

        for(int slip_i=0; slip_i<5*gp_data_.mean.size(); slip_i++) // HERE
        {
            P_pred = STM_*P_pred*STM_.transpose() + Q_;
            if ((slip_i % 5) ==0) //odomdata available //HERE
            {
            double chi0_slip=gp_data_.mean.at(i);
            double chi1_slip=gp_data_.mean.at(i)+gp_data_.sigma.at(i);
            double chi2_slip=gp_data_.mean.at(i)-gp_data_.sigma.at(i);

            double chi0_odo=0.8/(1.0-chi0_slip);
            double chi1_odo=0.8/(1.0-chi1_slip);
            double chi2_odo=0.8/(1.0-chi2_slip);

            double chi_UT_est=(chi0_odo+chi1_odo+chi2_odo)/3.0;
            double chi_UT_est_cov=((chi0_odo-chi_UT_est)*(chi0_odo-chi_UT_est)+(chi1_odo-chi_UT_est)*(chi1_odo-chi_UT_est)+(chi2_odo-chi_UT_est)*(chi2_odo-chi_UT_est))/3.0;

             R_IP_2<< std::max(0.03*0.03,chi_UT_est_cov*chi_UT_est_cov),0,0,0,
                      0,std::max(0.03*0.03,chi_UT_est_cov*chi_UT_est_cov),0,0,
                      0,0,std::max(0.05*0.05,chi_UT_est_cov*chi_UT_est_cov),0,
                      0,0,0,0.05*0.05;

             R_IP=25*R_IP_1*R_IP_2*R_IP_1.transpose();
             // R_IP=R_IP_1*R_IP_2*R_IP_1.transpose();


            K_pred=P_pred*H_.transpose()*(H_*P_pred*H_.transpose() +R_IP).inverse();
            P_pred=(Eigen::MatrixXd::Identity(15,15) - K_pred*H_)*P_pred*(Eigen::MatrixXd::Identity(15,15)-K_pred*H_).transpose()  + K_pred*R_IP*K_pred.transpose();
            // std::cout << "i:" << i << '\n';
            i++;
            }

            ins_enu_slip << GpPredictor::llh_to_enu(savePos[0],savePos[1],savePos[2]);
            ins_enu_slip_3p << GpPredictor::llh_to_enu(savePos[0]-3.0*sqrt(std::abs(P_pred(6,6))),savePos[1]-3.0*sqrt(std::abs(P_pred(7,7))), savePos[2]-3.0*sqrt(std::abs(P_pred(8,8))));
            ins_enu_slip3p << GpPredictor::llh_to_enu(savePos[0]+3.0*sqrt(std::abs(P_pred(6,6))),savePos[1]+3.0*sqrt(std::abs(P_pred(7,7))), savePos[2]+3.0*sqrt(std::abs(P_pred(8,8))));

            xy_errSlip = sqrt((ins_enu_slip3p(0)-ins_enu_slip(0))*(ins_enu_slip3p(0)-ins_enu_slip(0)) + (ins_enu_slip3p(1)-ins_enu_slip(1))*(ins_enu_slip3p(1)-ins_enu_slip(1)));
            ROS_ERROR_THROTTLE(0.5,"XYerror %.6f meters", xy_errSlip);
            // std::cout << "error" << '\n'<< xy_errSlip << '\n';

            if (xy_errSlip > 2.00)
            {

              ROS_ERROR("Stop Command Required, error is more than %.2f meters", xy_errSlip);
              ROS_ERROR("Stop command should be set at %u seconds after %.2f sec driving",i/10,odomUptCount/10.0);
              if (gp_arrived_time_ + i/10.0 - ros::Time::now().toSec()<0.0) // if the results from GP arrival time and the time for each odometry update sum is less then the current time stop immediately. This means delta time is negative--we needed to stop earlier.
              {
              // if (gp_arrived_time_ + i/10.0 <0.0) {
                stop_cmd_msg_.data = 0.5;
                cmd_stop_=stop_cmd_msg_.data;
              }
              else //otherwise calculate the necessary time for stopping -- when do we need to stop from now.
              {
                stop_cmd_msg_.data = gp_arrived_time_ + i/10.0 - ros::Time::now().toSec();
                // stop_cmd_msg_.data = gp_arrived_time_ + i/10.0 ;
                cmd_stop_=stop_cmd_msg_.data;

              }
              stop_cmd_pub_.publish(stop_cmd_msg_); //publish the delta time required to stop.
              ROS_DEBUG("Remaining Time to Stop = %.3f s", stop_cmd_msg_.data);

              break; // then break the for loop.
            }

        }

        new_gp_data_arrived_ = false; // Set the flag back to false, so that this does not happen again until new data comes in on the subscriber callback and sets this flag back to true
        i=0.0;
        slip_i=0.0;
        startRecording=stopRecording+ceil(cmd_stop_)*10+10+30; // Do we need a buffer time? Since we are stopping 3 seconds, and there could be some other time added to the process...idk...
        stopRecording=startRecording+150; //It should be 150...
        ROS_WARN("Start Next Recording at %.2f", startRecording/10);
        ROS_WARN("Stop Next Recording at %.2f", stopRecording/10);
        gp_flag =false; // DONT FORGET THIS IN HERE
    }

}

bool GpPredictor::LoadParameters(const ros::NodeHandle& nh_){
  if (!ros::param::get("init_llh/x", init_x)) return false;
  if (!ros::param::get("init_llh/y", init_y)) return false;
  if (!ros::param::get("init_llh/z", init_z)) return false;
  if (!ros::param::get("init_ecef/x", init_ecef_x)) return false;
  if (!ros::param::get("init_ecef/y", init_ecef_y)) return false;
  if (!ros::param::get("init_ecef/z", init_ecef_z)) return false;
  return true;
}

GpPredictor::Vector3 GpPredictor::llh_to_enu(double lat, double lon, double height){

        // countLLH2ENU++;
        // ROS_INFO_STREAM("countLLH2ENU: " << countLLH2ENU);
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gp_predictor");
  ros::NodeHandle nh("");

  GpPredictor gp_predictor(nh);

  ros::spin();

  return 0;
}
