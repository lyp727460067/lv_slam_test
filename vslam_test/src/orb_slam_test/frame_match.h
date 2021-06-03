#ifndef _FRAME_MATCH_H
#define  _FRAME_MATCH_H
#include "Eigen/Core"
#include "transfom.h"
#include "key_frame.h"
struct FrameMatchOption {};
class FrameMatch {
 public:
  FrameMatch(const FrameMatchOption &option);
  Rigid3d Match();
  private:
  void CorrelationFind();
};

#endif