#ifndef _IMAG_TRAKER_H
#define  _IMAG_TRAKER_H
#include "Eigen/Core"
#include "opencv2/opencv.hpp"

class MapInterface {
 public:
  virtual std::unique_ptr<MapInterface> GetData() = 0;
};

class LocalMap : public MapInterface {};
class GlobeMap : public MapInterface {};

class ImagTraker
{
  public:
  ImagTraker();
  void PoseExtrapolator(Eigen::Matrix4d &init_pose, const cv::Mat &target,
                        const cv::Mat &source, Eigen::Matrix4d *relate_pose);
  void TrakeMap(const Eigen::Matrix4d &init_pose, const cv::Mat &source,
               std::unique_ptr<MapInterface> map,Eigen::Matrix4d *relate_pose);

 private:  



};



#endif