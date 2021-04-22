
#include "pose_slide_window.h"

#include <vector>

#include "gtest/gtest.h"
namespace {
class PoseSldeWindonTest : public testing::Test {
 public:
 protected:
  void SetUp() { GenerateData(); }

  void GenerateData(void) {
    map_points.resize(10);
    int i = 0;
    for (auto& point : map_points) {
      i++;
      point.x = i;
      point.y = 3;
      point.z = i + 1;
    }
    tansforms_.resize(10);
    float yaw = 0;
    float t = 0;
    for (auto& trans : tansforms_) {

      trans = Eigen::Matrix4f::Identity();
      Eigen::Matrix3f r;
      r = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY());
      trans.block<3, 3>(0, 0) = r;
      trans(2, 3) = t;
       yaw += 0.02;
      t -= 0.1;     
    }

    Eigen::Matrix3f m_k;
    m_k << 385.0450439453125, 0.0, 323.1961975097656, 0.0, 385.0450439453125,
        244.11233520507812, 0.0, 0.0, 1.0;
    K = Eigen::Matrix4f::Identity();
    K.block<3, 3>(0, 0) = m_k;
  };
  cv::Point3f ProjectPoint(const Eigen::Matrix4f t,
                           const Eigen::Vector3f point) {
    Eigen::Vector4f m_point = Eigen::Vector4f::Identity();
    m_point.head<3>() = point;

    m_point = t * m_point;
    return {m_point.x(), m_point.y(), m_point.z()};
  }

  Eigen::Vector3f Cv2Eigen(const cv::Point3f& point) {
    return {point.x, point.y, point.z};
  }

  std::vector<Eigen::Matrix4f> tansforms_;
  std::vector<cv::Point3f> map_points;
  Eigen::Matrix4f K;
  PoseSlideWindow slinde_windows_;

  // const cv::Mat K =
  //     (cv::Mat_<float>(3, 3) << 385.0450439453125, 0.0, 323.1961975097656,
  //     0.0,
  //      385.0450439453125, 244.11233520507812, 0.0, 0.0, 1.0);
};
}  // namespace

TEST_F(PoseSldeWindonTest, TestInsert) {
  Frame last_frame;
  bool first = 0;
  for (int i = 0; i < 10; i++) {
    Eigen::Matrix4f kt = tansforms_[i];
    auto transform = tansforms_[i];
    int index = 0;
    Frame frame;
    for (auto map_point : map_points) {
      auto trace_point = ProjectPoint(kt.inverse(), Cv2Eigen(map_point));
      std::cout<<trace_point<<std::endl;
      auto preject_point = ProjectPoint(K, Cv2Eigen(trace_point));
      KeyPoint key_point;
      if (first == 0) {
        key_point.depth = trace_point.z;
        key_point.point = cv::Point2f{preject_point.x / trace_point.z,
                                      preject_point.y / trace_point.z};
        key_point.velocity = {0, 0};
      } else {
        key_point.depth = -1;
        key_point.point = cv::Point2f{preject_point.x / trace_point.z,
                                      preject_point.y / trace_point.z};
        cv::Point2f vel = key_point.point - last_frame.keyPoints_[i].point;
        key_point.velocity = {vel.x, vel.y};
      }
      std::cout << key_point.point << "z = " << key_point.depth << std::endl;
      frame.keyPoints_.insert({index, key_point});
      index++;
    }
    // if(first==0){
    frame.pose_q =
        Eigen::Quaterniond(transform.block<3, 3>(0, 0).template cast<double>());
    frame.pose_t = transform.block<3, 1>(0, 3).template cast<double>();
    std::cout<<frame.pose_t<<std::endl;
    // }
    first = true;

    last_frame = frame;
    slinde_windows_.Insert(frame);
    // std::cout<<"insert frame"<<frame.keyPoints_[0].point<<std::endl;
  }

  auto pose_vet = slinde_windows_.GetSlidesPose();
  for (auto pose : pose_vet) {
    std::cout << pose << std::endl;
  }
}
