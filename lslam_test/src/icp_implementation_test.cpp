
#include "gtest/gtest.h"
#include "icp_implementation.h"
#include "Eigen/Geometry"
#include "pl_icp.h"
#include "random"
#include "grid_map.h"
namespace {
class IcpImplementationTest : public testing::Test {
 protected:
  OriginalIcp original_icp_;
  PlIcp  pl_icp_;
  OccupancyGridMap  grid_map_;
};

TEST_F(IcpImplementationTest, MatchTest) {
  std::vector<Eigen::Vector3f> source = {{1, 0, 0},   {1.5, 0, 0}, {2., 0, 0},
                                           {2.5, 0, 0}, {2.8, 0, 0}, {3, 0, 0}};
  Eigen::Matrix3f R;
  R = Eigen::AngleAxis<float>(0.5, Eigen::Vector3f(0, 0, 1)); Eigen::Vector3f t(1, 0, 0); 
  static std::default_random_engine dre;
  std::vector<Eigen::Vector3f> target_points;
  for (auto &point : source) {
    Eigen::Vector3f target_point = R * point;
    std::uniform_real_distribution<float> di(-0.1, 0.1);
    //target_point.x() += di(dre);
    //target_point.y() += di(dre);
    // target_point.z() = 0;// += di(dre);
    target_point += t;
    target_points.push_back(target_point);
  }
  Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f pose_estimate;
  original_icp_.Match(IcpInterface::Option{}, init_pose, source, target_points, pose_estimate);

  Eigen::Matrix3f diff = R.transpose() *pose_estimate.block<3, 3>(0, 0);
  EXPECT_NEAR(diff(0, 0), 1, 0.01);
  EXPECT_NEAR(diff(1, 1), 1, 0.01);
  EXPECT_NEAR(diff(2, 2), 1, 0.01);
  float trance_err = (t - init_pose.block<3, 1>(0, 3)).norm();
  EXPECT_NEAR(trance_err, 1.0, 0.1);


}
TEST_F(IcpImplementationTest, MatchCeresTest) {
  std::vector<Eigen::Vector3f> source = {{0.1, 0, 0},  {2, 0, 0},{0.3, 0, 0}, {0.4, 0, 0},  {1.5, 0, 0}, {1.5, 0.1, 0},
                                         {1.5, 0.4, 0}, {1.5, 0.8, 0}, {3, 0, 0}};
  Eigen::Matrix3f R;
  R = Eigen::AngleAxis<float>(0.5, Eigen::Vector3f(0, 0, 1));
  Eigen::Vector3f t(1, 0, 0);
  
  static std::default_random_engine dre;
  std::vector<Eigen::Vector3f> target_points;
  for (auto &point : source) {
    Eigen::Vector3f target_point = R * point;
    std::uniform_real_distribution<float> di(-0.05, 0.05);
   target_point.x() += di(dre);
   target_point.y() += di(dre);
    target_point.z() = 0;// += di(dre);
    target_point += t;
    target_points.push_back(target_point);
  }
  Eigen::Matrix3f init_r ;
  init_r  =  Eigen::AngleAxis<float>(0.1,Eigen::Vector3f(0,0,1));
  Eigen::Vector3f init_t =  {0.7,0,0};
  Eigen::Matrix4f init_pose =  Eigen::Matrix4f::Identity() ;
  init_pose.block<3,3>(0,0) =  init_r;
  init_pose.block<3,1>(0,3) =  init_t;

  Eigen::Matrix4f pose_estimate;
  pl_icp_.Match(IcpInterface::Option{}, init_pose, source, target_points, pose_estimate);

  Eigen::Matrix3f diff = R.transpose() *pose_estimate.block<3, 3>(0, 0);
  EXPECT_NEAR(diff(0, 0), 1, 0.01);
  EXPECT_NEAR(diff(1, 1), 1, 0.01);
  EXPECT_NEAR(diff(2, 2), 1, 0.01);
  float trance_err = (t - pose_estimate.block<3, 1>(0, 3)).norm();
  EXPECT_NEAR(trance_err, 0.0, 0.01);

}


}