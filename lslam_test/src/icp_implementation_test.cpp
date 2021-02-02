#include "gtest/gtest.h"
#include "icp_implementation.h"
#include "Eigen/Geometry"
#include "random"
namespace {
  class IcpImplementationTest:public testing::Test
  {
    protected:
    OriginalIcp  original_icp_;
  };

TEST_F(IcpImplementationTest,MatchTest){
  std::vector<Eigen::Vector3f>  source = 
  {
    {1,0,0},
    {1.5,0,0},
    {2.,0,0},
    {2.6,0,0},
    {2.7,0,0},
    {3,0,0}
  };
  Eigen::Matrix3f R;
  R = Eigen::AngleAxis<float>(0.5, Eigen::Vector3f(0, 0, 1)); 
  Eigen::Vector3f t(1,0,0);
  std::cout<<R<<std::endl;
  static std::default_random_engine dre;
  std::vector<Eigen::Vector3f> target_points;
  for (auto &point : source) {
    Eigen::Vector3f target_point = R * point;
    std::uniform_real_distribution<float> di(-0.1, 0.1);
    //target_point.x() += di(dre);
    //target_point.y() += di(dre);
    //target_point.z() = 0;// += di(dre);
    target_point+=t;
    target_points.push_back(target_point);
  }
  Eigen::Matrix4d init_pose  =  Eigen::Matrix4d::Identity();
  original_icp_.Match(init_pose,source,target_points);
  Eigen::Matrix3d expect_r  =  init_pose.block<3,3>(0,0);
  std::cout<<expect_r<<std::endl;
  EXPECT_EQ(expect_r,R.cast<double>());
}




}