#ifndef _IMAG_TRAKER_H
#define  _IMAG_TRAKER_H
#include "Eigen/Core"
#include "opencv2/opencv.hpp"
class ImagTraker
{
  public:
  ImagTraker();
  void AddImage(cv::Mat image);
  bool LocalMatch(Eigen::Matrix4d &init_pose, 
  
  )



};



#endif