#ifndef _POSE_SLIDE_WINDOW_H
#define  _POSE_SLIDE_WINDOW_H
#include "key_frame.h"
#include <map>
#include <vector>
#include "opencv/opencv.hpp"
#include <memory>
#include "opencv2/core.hpp"

class Frame
{
  public:
  int id_;
  std::vector<std::pair<int,cv::Point2d>> features_;
  
};

struct SliedeWindowResult
{



};

class PoseSlideWindow
{
  public:
  SliedeWindowResult  Insert(std::unique_ptr<Frame> frame)
  {




  };




  private:
  int window_size  = 10;


  std::vector<Frame*> frames_(window_size,nullptr);
  std::map<>



}




#endif
