
#include <core_navigation/ICEclass.h>
#include <libcluster/merge.h>
#include <libcluster/probutils.h>
#include <libcluster/libcluster.h>
#include <libcluster/distributions.h>

// STD
#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <numeric>

using namespace libcluster;
using namespace distributions;
using namespace merge;
using namespace std;

// This is the constructor

ICEclass::ICEclass(){          // residuals in corenav are 4 dimensional

  is_outlier = false;
  res_count = -1;
  all_res_count = 0;
  for (int k =0; k<500; k++){
    num_obs.push_back(0);
  };

  residuals.setZero(500,4);

  // How to initialize the globalMixtureModel ?

  Eigen::RowVectorXd m(4);
  m << 0.0000001, 0.0000005, 0.0000003, 0.0000001;   // set the mean properly
  Eigen::MatrixXd c(4,4);
  Eigen::MatrixXd c1(4,4);
  Eigen::MatrixXd c2(4,4);
  double T_r_ = 0.685;
  c1<<0.5,0.5,0.0,0.0,
  1/T_r_, -1/T_r_, 0.0,0.0,
  0.0, 0.0,1.0,0.0,
  0.0,0.0,0.0,1.0;
  c2<<0.03*0.03, 0.0,0.0,0.0,
  0.0, 0.03*0.03, 0.0,0.0,
  0.0,0.0,0.05*0.05,0.0,
  0.0,0.0,0.0,0.05*0.05;

  c<< 25*c1*c2*c1.transpose();
  globalMixtureModel.push_back(boost::make_tuple(0, 0, 0.0, m, c));

  cout << "ICE ICE baby.. \n"<< endl;
  //cout << is_outlier <<endl;
}


void ICEclass::icefunc(Eigen::RowVectorXd res, int ind){

          num_obs.at(ind) = num_obs.at(ind)+1;

        }


void ICEclass::merging(Eigen::RowVectorXd res){

            // if (res_count == 99) // set the threshold for group size of outliers to be put into variational clustering
            // {
                    // cout << "The residual matrix -- before merging " << "\n" << residuals << "\n" << endl;
            residuals.block(res_count,0,1,4) = res;

            if (res_count == 499){
                  StickBreak weights;
                  vector<GaussWish> clusters;
                  Eigen::MatrixXd qZ;

                  learnVDP(residuals, qZ, weights, clusters);    //variational dirichlet process
                  // cout << "Total number of new inlier observations analysed before merging -- " << std::accumulate(num_obs.begin(),num_obs.end(),0) << endl;
                  // update the number of obs in each component
                  cout << " Merging ------- " << endl;
                  globalMixtureModel = updateObs(globalMixtureModel, num_obs);
                  std::fill((num_obs).begin(), (num_obs).end(), 0);

                  // merge the curr and prior mixture models.
                  globalMixtureModel = mergeMixtureModel(residuals, qZ, globalMixtureModel, clusters, weights, 0.05, 20);


                  // cout << " --------------------------- Resetting the residuals matrix , looking for new batch of residuals ------------------------------- \n\n" << endl;


                  (residuals).setZero(500,4); // reset residuals matrix and the res count
                  res_count = -1;
          }

            //}



      }

// void ICEclass::icefunc(Eigen::RowVectorXd res, int ind){
//
//           num_obs.at(ind) = num_obs.at(ind)+1;
          //is_outlier = false;
          // int ind(0);
          // double prob, probMax(0.0);
          // Eigen::MatrixXd cov_min(4,4);     // is the size okay ?
          // Eigen::RowVectorXd mean_min(4);
          //Eigen::VectorXd res(4);

          //ob_count = ob_count + 1;
          //all_res_count += 1;
          // cout << " Total number of residuals  -- " << all_res_count << endl;

          // for(int k=0; k<globalMixtureModel.size();k++){
          //
          //   merge::mixtureComponents mixtureComp = globalMixtureModel[k];
          //
          //   Eigen::RowVectorXd mean = mixtureComp.get<3>();
          //
          //   double quadform  = (res-mean)* (mixtureComp.get<4>()).inverse() * (res-mean).transpose();
          //   double norm = std::pow(std::sqrt(2 * M_PI),-1) * std::pow((mixtureComp.get<4>()).determinant(), -0.5);
          //
          //   prob =  norm * exp(-0.5 * quadform);
          //
          //   if (prob >= probMax)
          //   {
          //           ind = k;
          //           probMax = prob;
          //           cov_min = mixtureComp.get<4>();
          //           mean_min = mixtureComp.get<3>();
          //   }
          // }
          //   //res_os << res(0) << " " << res(1) << " " << res(2) << " " << res(3) << endl;

            // Use z-test to see if residuals is considered an outlier
            // double z_1 = ((res(0) -mean_min(0))/std::sqrt(cov_min(0,0)));
            // double z_2 = ((res(1)-mean_min(1))/std::sqrt(cov_min(1,1)));
            // double z_3 = ((res(2)-mean_min(2))/std::sqrt(cov_min(2,2)));
            // double z_4 = ((res(3)-mean_min(3))/std::sqrt(cov_min(3,3)));

            // double mahalcost  = 0.0;
            // mahalcost = (res-mean_min)*cov_min.inverse()*(res-mean_min).transpose();
            // double critical_valchi4 = 9.488;
            //
            // // only consider residuals more than 'n' stds from model
            // if (mahalcost > critical_valchi4){ // outlier
            //
            //++res_count; // increment the number of outlier residuals
            //     // cout << "outlier" << res_count <<endl;
            //     // residuals(res_count,0) = res(0);
            //     // residuals(res_count,1) = res(1);
            //     // residuals(res_count,2) = res(2);
            //     // residuals(res_count,3) = res(3); // put the residuals in the matrix
            //     //res_out_os << res(0) << " " << res(1) << " " << res(2) << " " << res(3) << endl;
            //     //ob_count -= 1;
            //     is_outlier = true;
            //   }
            // else{ // inlier
             // increase the number of inlier in that component
            //     // cout << "inlier"<< endl;
            // }
        // }
