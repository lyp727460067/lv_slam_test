#ifndef _POSE_SLIDE_WINDOW_H
#define  _POSE_SLIDE_WINDOW_H
#include "key_frame.h"
#include <map>
#include <vector>
#include "opencv/opencv.hpp"

class Frame
{
  public:
  std::map<int,cv::Vector2d> features_;
};

class PoseSlideWindow
{
  public:


  private:
  int window_size  = 10;
  std::vector<Frame*> frames_(window_size,nullptr);
  



}




#endif
