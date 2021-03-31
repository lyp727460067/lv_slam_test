#ifndef _POSE_SLIDE_WINDOW_H
#define  _POSE_SLIDE_WINDOW_H
#include "key_frame.h"
#include <map>
#include <vector>
#include "opencv/opencv.hpp"
#include <memory>
#include "opencv2/core.hpp"

struct KeyPointWithId {
  int id;
  cv::Point2d point;
  double depth=-1;
};
class Frame {
 public:
  int id_;
  std::vector<KeyPointWithId> key_points_with_ids_;
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
  std::vector<std::unique_ptr<Frame>> frames_(window_size,nullptr);
  std::multimap<int,Frame>  key_point;


};




#endif
