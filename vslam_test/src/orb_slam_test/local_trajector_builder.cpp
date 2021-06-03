#include "local_trajector_builder.h"
#include "glog/logging.h"
// LocalTrajectorBuilder::LocalTrajectorBuilder(
//     const LocalTrajectorBuilderOption option) {}

// void LocalTrajectorBuilder::AddSenorData(const sensor::ImageData& image) {
  
//   if(track_pre_frame_==nullptr){
//      auto keypoints_with_des =
//       ExtractorFeatureDes(std::vector<const sensor::ImageData>{image});   
//       track_pre_frame_ =  std::make_unique<TrackPreFrame>(keypoints_with_des);
//       std::shared_ptr<KeyFrame> key_frame =  std::make_shared<KeyFrame>(keypoints_with_des);
//       key_frames_.emplace(0,key_frame) ;
//       local_mapping_->Insert(key_frame);
//       LOG(INFO)<<"init pre_frame";
//       return ;
//   }
// //   auto track_frame_data =  track_pre_frame_->track(image);
// //   //没有跟踪的地方提取特征点，表述子，双目计算出深度点
// //   auto extract_frame_data  =   ExtractorFeatureDes(image);
// //   track_frame_data->data+=extract_frame_data->data;
// //   if(track_frame_data->size()<option.track_pre_frame_num){

// //   }



// }




//  std::shared_ptr<FrameData> LocalTrajectorBuilder::ExtractorFeatureDes(
//     const  sensor::ImageData& imags) {

// }