#ifndef _EKF_
#define _EKF_
#include "Eigen/Eigen"
#include <chrono>
#include <opencv/core.hpp>
struct ImuData {
  uint64_t time;
  Eigen::Vector3f gry;
  Eigen::Vector3f acc;
};
struct OdomData {
  uint64_t time;
  Eigen::Vector2f tranlation;
  float theta;
};

class Inertia {
 public:
  std::vector<ImuData> imu_data_;

  void Advance(uint64_t times) {}
};

struct EkfStates {
  Eigen::Quaternionf q;
  Eigen::Vector3f p;
  Eigen::Vector3f v;
  Eigen::Vector3f gb;
  Eigen::Vector3f ab;
};

class EkfTest {
 public:
  void predict();
  Eigen::Matrix34f update();
  void PushData(ImuData imu) { imu_data_.push_back(imu); }
  void PushData(OdomData odom) { odom_data_.push_back(odom); }
  void PushData(const cv::Mat& img) {}

 private:
  std::vector<ImuData> imu_data_;
  std::vector<OdomData> odom_data_;
};

#endif