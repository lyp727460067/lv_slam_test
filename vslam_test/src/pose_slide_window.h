#ifndef _POSE_SLIDE_WINDOW_H
#define  _POSE_SLIDE_WINDOW_H
#include "key_frame.h"
#include <map>
#include <vector>
#include "opencv/opencv.hpp"
#include <memory>
#include "opencv2/core.hpp"
#include "Eigen/Eigen"

struct KeyPoint {
  cv::Point2d point;
  double depth=-1;
};
class Frame {
 public:
  std::map<int,KeyPoint> keyPoints_;

};

struct SliedeWindowResult
{



};

class PoseSlideWindow
{
  public:
  SliedeWindowResult  Insert(std::unique_ptr<Frame> frame)
  {
   frames_.i 
  };




  private:
  static int frame_id; 
  int window_size  = 10;
  std::map<int,std::unique_ptr<Frame>> frames_;
  std::multimap<int,Frame>  key_point_id_Corresponding_;


};




#endif
