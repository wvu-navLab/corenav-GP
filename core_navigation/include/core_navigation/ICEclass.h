
#ifndef iceclass_H
#define iceclass_H


#include <libcluster/merge.h>
#include <libcluster/probutils.h>
#include <libcluster/libcluster.h>
#include <libcluster/distributions.h>

// STD
#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>


using namespace libcluster;
using namespace distributions;
using namespace merge;
using namespace std;



class ICEclass
{
public:


  // each component has the  form tuple( Total num. obs, Num obs in comp., comp. weight, compt. mean, compt. cov.)

  bool is_outlier;
  int all_res_count;
  std::vector<mixtureComponents> globalMixtureModel; // the mixture model with each component
  int res_count; // Number of outliers to be taken before merging
  std::vector<int>num_obs; // number of inlier residuals in each component
  Eigen::MatrixXd residuals; // matrix containing the outlier residuals
  ICEclass();


  // add constructor

  void icefunc(Eigen::RowVectorXd res);
  void merging(Eigen::RowVectorXd res); // returns the merged mixture model or the same mixture model in case of inlier


};

#endif
