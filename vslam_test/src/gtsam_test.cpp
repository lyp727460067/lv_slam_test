#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include  "gtsam/geometry/Pose2.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Values.h"
#include <iostream>
#include "gtsam/nonlinear/Marginals.h"
using namespace std;

void test1()
{
  gtsam::NonlinearFactorGraph graph;
  gtsam::Pose2 prior_mean(0.0, 0.0, 0.0);
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));
  graph.add(gtsam::PriorFactor<gtsam::Pose2>(1,prior_mean,prior_noise));


  gtsam::Pose2  odomtry(2.0, 0.0, 0.0);
  gtsam::noiseModel::Diagonal::shared_ptr odomtry_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));

  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(1,2,odomtry,odomtry_noise));
  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(2,3,odomtry,odomtry_noise));
  graph.print("\nFactor Graph:\n");
  gtsam::Values initial;
  initial.insert(1,gtsam::Pose2(0.5,0.0,0.2));
  initial.insert(2,gtsam::Pose2(2.3,0.1,-0.2));
  initial.insert(3,gtsam::Pose2(4.1,0.1,0.1));
  initial.print("\n Initial Estimate: \n");
  
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph,initial).optimize();
  result.print("Final Result:\n");
  std::cout.precision(2) ; 

  gtsam::Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

}

int main(int argc,char ** argv)
{
  test1();
  return 0;
}
