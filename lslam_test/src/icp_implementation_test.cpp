#include "gtest/gtest.h"
#include "icp_implementation.h"
#include "Eigen/Geometry"
#include "random"
namespace {
class IcpImplementationTest : public testing::Test {
 protected:
  OriginalIcp original_icp_;
};

TEST_F(IcpImplementationTest, MatchTest) {
  std::vector<Eigen::Vector3f> source = {{1, 0, 0},   {1.5, 0, 0}, {2., 0, 0},
                                         {2.6, 0, 0}, {2.7, 0, 0}, {3, 0, 0}};
  Eigen::Matrix3f R;
  R = Eigen::AngleAxis<float>(0.5, Eigen::Vector3f(0, 0, 1));
  Eigen::Vector3f t(1, 0, 0);
  static std::default_random_engine dre;
  std::vector<Eigen::Vector3f> target_points;
  for (auto &point : source) {
    Eigen::Vector3f target_point = R * point;
    std::uniform_real_distribution<float> di(-0.1, 0.1);
    target_point.x() += di(dre);
    target_point.y() += di(dre);
    // target_point.z() = 0;// += di(dre);
    target_point += t;
    target_points.push_back(target_point);
  }
  Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();

  original_icp_.Match(IcpInterface::Option{}, init_pose, source, target_points);

  Eigen::Matrix3f diff = R.transpose() * init_pose.block<3, 3>(0, 0);
  EXPECT_NEAR(diff(0, 0), 1, 0.01);
  EXPECT_NEAR(diff(1, 1), 1, 0.01);
  EXPECT_NEAR(diff(2, 2), 1, 0.01);
  float trance_err = (t - init_pose.block<3, 1>(0, 3)).norm();
  EXPECT_NEAR(trance_err, 0.0, 0.1);
}
}