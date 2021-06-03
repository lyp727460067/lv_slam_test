#ifndef _LOCAL_TRAJECTOR_BUILD_H
#define _LOCAL_TRAJECTOR_BUILD_H

#include <Eigen/Core>
#include <map>
#include <memory>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "local_mapping_window.h"
#include "motion_filter.h"
#include "frame_match.h"
#include "key_frame.h"
#include "key_frame_correlation.h"
namespace sensor
{
	struct ImageData{
		uint64_t time;
		cv::Mat image;
	};


}

struct LocalTrajectorBuilderOption
{
  int track_pre_frame_num =  20;


};
class TrackPreFrame {
 public:
//   TrackPreFrame(std::shared_ptr<FrameData> frame_data)
//       : pre_frame_data_(std::move(frame_data)) {}

//   std::unique_ptr<FrameData> track(const sensor::ImageData& image);

//  private:
//   std::shared_ptr<FrameData> pre_frame_data_;
// };
// class LocalTrajectorBuilder {
//  public:
//   LocalTrajectorBuilder(const LocalTrajectorBuilderOption option);
//   void AddSenorData(const sensor::ImageData& image);
  
//  private:
//   LocalTrajectorBuilderOption option;
//   std::shared_ptr<FrameData> LocalTrajectorBuilder::ExtractorFeatureDes(
//     const sensor::ImageData& imags);
//   std::map<int, KeyFrame> key_frames_;

//   std::unique_ptr<LocalMappingWindow> local_mapping_;
//   std::unique_ptr<MotionFilter> motion_filter_;
//   std::unique_ptr<FrameMatch> frame_match_;
// 	std::unique_ptr<KeyFrameCorrection> keyframe_correction_;
 
//   std::unique_ptr<TrackPreFrame> track_pre_frame_;

};

#endif