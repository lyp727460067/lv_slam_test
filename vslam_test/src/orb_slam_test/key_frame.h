#ifndef _KEY_FRAME_H
#define  _KEY_FRAME_H

#include <Eigen/Core>
#include <map>
#include <memory>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include <map>

struct ImagInfoData {
  std::map<int, cv::KeyPoint> key_points;
  std::map<int, cv::Mat> describe;
};

struct ImagData {
  std::map<int, ImagInfoData> info_datas;
};

struct MapPointsData {
  std::map<int, Eigen::Vector3f> points;
  std::map<int, cv::Mat> describe;
};

class KeyFrame {
 public:
 private:
};

#endif