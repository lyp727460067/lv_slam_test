#ifndef _STEREO_TEST_H
#define _STEREO_TEST_H
#include "Eigen/Core"
#include <iostream>
#include <map>
#include <vector>
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv.hpp"
#include "pose_slide_window.h"
#include "random"

class FrameImag
{
  public:
  Frame frame;
  cv::Mat img;
};
class VTrack
{
  public:
   std::map<int, KeyPoint> OpticalTracking(cv::Mat init_pose, FrameImag* pre_frame,
                                    FrameImag* cur_frame) {
     if (pre_frame == nullptr) {
       return std::map<int, KeyPoint>();
     }

     if (!init_pose.empty()) {
     }
     std::vector<uint8_t> status;
     std::vector<float> err;
     std::vector<cv::Point2f> pre_frame_point =
         KeyPointToCvPoints(pre_frame->frame.keyPoints_);
      
     std::vector<cv::Point2f> curr_frame_point;
     std::cout<<"calcOpticalFlowPyrLK"<<std::endl;
     std::cout<<"pre_frame_point.size()"<<pre_frame_point.size()<<std::endl;
     cv::calcOpticalFlowPyrLK(pre_frame->img, cur_frame->img, pre_frame_point,
                              curr_frame_point, status, err, cv::Size(21, 21),
                              1);
     auto pre_frame_inter = pre_frame->frame.keyPoints_.begin();
     std::cout<<"track_points"<<std::endl;
     std::map<int, KeyPoint> track_points;
     for (int i = 0; i < status.size(); i++) {
       if (status[i]) {
         int id = pre_frame_inter->first;
         cv::Point2f velocity = {curr_frame_point[i]-pre_frame_point[i]  };
         track_points.emplace(
             id, KeyPoint{curr_frame_point[i], {velocity.x, velocity.y}});
       }
        pre_frame_inter++;   
     }

     std::cout<<" RemoveOutLiner"<<std::endl;
    return RemoveOutLiner(pre_frame->frame.keyPoints_,track_points);

   }
   std::map<int, KeyPoint> RemoveOutLiner(const std::map<int, KeyPoint>& pre_points,
                       const std::map<int, KeyPoint>& cur_points) {
     std::vector<cv::Point2f> cv_cur_points = KeyPointToCvPoints(cur_points);
     std::vector<cv::Point2f> cv_pre_points;
     for(auto point:cur_points){
        cv_pre_points.push_back(  pre_points.at(point.first).point);
     }
    std::cout<<"cv_cur_points"<<std::endl;   
     std::map<int, KeyPoint> track_points;
     std::vector<uint8_t> status;
     cv::Mat F = cv::findFundamentalMat(cv_pre_points, cv_cur_points,
                                        cv::FM_RANSAC, 1.0, 0.99, status);

     std::cout<<"findFundamentalMat"<<std::endl;
    int  i  = 0;
    for (auto point : cur_points) {
      if (status[i++]) {
        track_points.emplace(point);
      }
    }

     return track_points;
   }
 
   std::vector<cv::Point2f> RemoveWithStates(
       const std::vector<cv::Point2f>& points,
       const std::vector<uint8_t>& states) {
     std::vector<cv::Point2f> points_with_states;
     for (int i = 0; i < states.size(); i++) {
       if (states[i]) {
         points_with_states.push_back(points[i]);
       }
     }
     return points_with_states;
   }
    std::vector<cv::Point2f> KeyPointToCvPoints(const std::map<int,KeyPoint>& key_points){
      std::vector<cv::Point2f> cv_points;
      for(auto point:key_points){
        cv_points.push_back(point.second.point);
      }
      return cv_points;
    }

  cv::Mat GetMask(const cv::Mat& cam,const std::map<int, KeyPoint>& points){

    cv::Mat mask = cv::Mat(cam.size(), CV_8UC1, cv::Scalar(100));
    for (auto point : points) {
      cv::Point2f cvpoint = point.second.point;
      if (mask.at<uint8_t>(cvpoint.y, cvpoint.x)) {
        const int min_feat_dist = 10;
        cv::circle(mask,cvpoint, min_feat_dist, cv::Scalar(0), cv::FILLED);
      }
    }
      return mask;
  }

  void ShowTracker(FrameImag *frame)
  {
      cv::Mat img_feat;
      cv::cvtColor(frame->img, img_feat,  cv::COLOR_GRAY2RGB);
      for(auto keypoint:frame->frame.keyPoints_){
          //cv::line(img_feat,cur_feats[i],pre_feats[i],cv::Scalar(0,0,255));
          cv::circle(img_feat,keypoint.second.point,1,cv::Scalar(0,255,0));   
      }
     cv::imshow("tracker show",img_feat);
     cv::waitKey(1);

  }

  Eigen::Matrix4d Tracking(const cv::Mat& left_cam, const cv::Mat& right_cam)
  {
    if(pre_img!=nullptr){
    std::cout<<"pre_imgpre_img->frame.keyPoints_.size()"<<pre_img->frame.keyPoints_.size()<<std::endl;
    }
    FrameImag *curr_img = new FrameImag{Frame(),left_cam};
    std::cout<<"Tracking"<<std::endl;
    auto track_points  = OpticalTracking(cv::Mat(),pre_img,curr_img);


    curr_img->frame.keyPoints_ = track_points; 
    ShowTracker(curr_img);

    if(track_points.size()<20){
      std::cout<<"track_points.size"<<track_points.size()<<std::endl;
      FrameImag *right_img = new FrameImag{Frame(),right_cam};
      auto ex_feature = ExtraFeature(GetMask(left_cam,track_points),left_cam,1000);
      std::cout<<"ExtraFeature"<<ex_feature.size()<<std::endl;
      track_points.insert(ex_feature.begin(),ex_feature.end());
      std::cout<<"track_points.size()"<<track_points.size()<<std::endl;
      curr_img->frame.keyPoints_ = track_points;   
      auto right_track_points =   OpticalTracking(cv::Mat(),curr_img,right_img);
      std::cout<<"tTriangulateTwoframe"<<std::endl;
      TriangulateTwoframe(Eigen::Matrix4f::Identity(),curr_img->frame.keyPoints_,right_track_points); 
      delete right_img;   
 
    }
    Frame curr_frame = {curr_img->frame.keyPoints_};
    std::cout<<"insert slide window"<<std::endl;
    auto pose_end =  pose_slide_window_.Insert(curr_frame);
    std::cout<<"end"<<std::endl;
    if(pre_img != nullptr){
      delete pre_img;
    }
    pre_img = curr_img;
   
    return pose_end.pose_end_;

  }
  Eigen::Vector2f CvPoint2Eigen(cv::Point2f& point){
    return Eigen::Vector2f{point.x,point.y};
  }
  Eigen::Matrix4f RightCamPose(){
    Eigen::Matrix4f pose =  Eigen::Matrix4f::Identity();
    pose(2,3)  =  0.05;
    return pose;
  }
  void TriangulateTwoframe(Eigen::Matrix4f pose, std::map<int, KeyPoint>& left,
                           const std::map<int, KeyPoint>& right) {
    for (auto& point : left) {
      if (point.second.depth == -1) {
        int left_id = point.first;
        cv::Point2f left_point = point.second.point;
        if(right.count(left_id)==0)continue;
        cv::Point2f right_point = right.at(left_id).point;
        auto point_3d = TriangulateTwoPoint(
            Eigen::Matrix4f::Identity(), RightCamPose(),
            CvPoint2Eigen(left_point), CvPoint2Eigen(right_point));
        point.second.depth = point_3d.z();
        std::cout<<" point_3d.z()"<< point_3d.z()<<std::endl;
      }
    }
  }
  Eigen::Vector3f TriangulateTwoPoint(const Eigen::Matrix4f& pose0,
                                        const Eigen::Matrix4f& pose1,
                                        const Eigen::Vector2f& point00,
                                        const Eigen::Vector2f& point11) {
    Eigen::Matrix4f M = Eigen::Matrix4f::Zero();
    Eigen::Vector2f point0{
        (point00.x() - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (point00.y() - K.at<double>(1, 2)) / K.at<double>(1, 1)};
    Eigen::Vector2f point1{
        (point11.x() - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (point11.y() - K.at<double>(1, 2)) / K.at<double>(1, 1)};

    //std::cout<<point0<<point1<<std::endl;
    M.row(0) = point0.x() * pose0.row(2) - pose0.row(0);
    M.row(1) = point0.y() * pose0.row(2) - pose0.row(1);
    M.row(2) = point1.x() * pose1.row(2) - pose1.row(0);
    M.row(3) = point1.y() * pose1.row(2) - pose1.row(1);
    Eigen::Vector4f point =
        M.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    return point.block<3,1>(0,0)/point(3);
  }
  std::map<int, KeyPoint> ExtraFeature(cv::Mat&&mask,const cv::Mat& img,int feature_num)
  {
    static int point_id  = 0;
    std::vector<cv::Point2f> feats_points;
    cv::goodFeaturesToTrack(img, feats_points, feature_num, 0.01, 30, mask);
    std::map<int, KeyPoint> track_points;
    for(auto point:feats_points){
      track_points.emplace(point_id++,KeyPoint{point, {}, -1});
    }
    return track_points;
  }

  bool IsInited(){
    return pre_img != nullptr;
  }
  private:
   FrameImag* pre_img;
   PoseSlideWindow pose_slide_window_;
    const cv::Mat K =
       (cv::Mat_<double>(3, 3) << 385.0450439453125, 0.0, 323.1961975097656,
        0.0, 385.0450439453125, 244.11233520507812, 0.0, 0.0, 1.0);

};

class  StereoTrack
{
  public:
   Eigen::Matrix4d Track(const cv::Mat& left_cam, const cv::Mat& right_cam) {
    return  tracker_.Tracking(left_cam,right_cam);


    //  if (states == 0) {
    //    Init(left_cam, right_cam);
    //    states = 1;
    //  } else if (states == 1) {
    //    if (!TrackKeyFrame(key_frame, left_cam)) {
    //      Init(left_cam, right_cam);
    //    }
    //  }
     //return pose_;
   }
   std::vector<cv::Point3f> GetTrackPoints() {
     Eigen::Matrix4d TrackPoseToPose = k_pose_.inverse() * pose_;
     std::vector<cv::Point3f> return_points;
     for (auto point : track_3dpoints) {
       Eigen::Vector4d poin = TrackPoseToPose.inverse() *
                              Eigen::Vector4d(point.x, point.y, point.z, 1);
       return_points.push_back({poin[0], poin[1], poin[2]});
     }
     return return_points;
   }
   cv::Mat GetVisuImag() { return feats_img; }

  private:
   void MatchesDescriber(const cv::Mat& left_cam, const cv::Mat& right_cam) {
     std::vector<cv::KeyPoint> keypoints1, keypoints2;
     cv::Mat descriptors1, descriptors2;
     cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
     cv::Ptr<cv::DescriptorExtractor> desctriptor_ex = cv::ORB::create();
     cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

     detector->detect(left_cam, keypoints1);
     detector->detect(right_cam, keypoints2);
     desctriptor_ex->compute(left_cam, keypoints1, descriptors1);
     desctriptor_ex->compute(right_cam, keypoints2, descriptors2);

     std::vector<cv::DMatch> matches;
     matcher->match(descriptors1, descriptors2, matches);

     double min_dist = 10000, max_dist = -1;

     for (int i = 0; i < descriptors1.rows; i++) {
       double dist = matches[i].distance;
       if (dist < min_dist) {
         dist = min_dist;
       }
       if (dist > max_dist) {
         dist = max_dist;
       }
     }

     std::vector<cv::DMatch> good_matches;
     for (int i = 0; i < descriptors1.rows;i++) {
       if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
         good_matches.push_back(matches[i]);
       }
     }

     for (int i = 0; i < good_matches.size(); i++) {
     }



    }
    void Init(const cv::Mat& left_cam, const cv::Mat &right_cam) {
      std::vector<cv::KeyPoint> left_keypoints;
      std::vector<cv::Point2f> left_feats, right_feats;
      int thresh = 10;
      cv::Mat mask = cv::Mat(left_cam.size(), CV_8UC1, cv::Scalar(100));
      cv::TermCriteria criteria = cv::TermCriteria(
          (cv::TermCriteria::MAX_ITER) + (cv::TermCriteria::EPS), 30 , 0.01);
       cv::goodFeaturesToTrack(left_cam, left_feats, 1000, 0.01, 30, cv::Mat());

      //cv::cornerSubPix(left_cam, left_feats, cv::Size(5, 5), cv::Size(-1, -1),criteria); 
 
      // cv::Ptr<cv::FastFeatureDetector> detector =
      //     cv::FastFeatureDetector::create(thresh);
      // detector->detect(left_cam, left_keypoints);
      // std::sort(left_keypoints.begin(), left_keypoints.end(),
      //           [](const cv::KeyPoint& p1, const cv::KeyPoint& p2) {
      //             return p1.response > p2.response;
      //           });

      // for (cv::KeyPoint keypoint : left_keypoints) {
      //   if (left_feats.size() < max_feats_size &&
      //       mask.at<uint8_t>(keypoint.pt.y, keypoint.pt.x)) {
      //     left_feats.push_back(keypoint.pt);
      //     cv::circle(mask, keypoint.pt, min_feat_dist, cv::Scalar(0),
      //                cv::FILLED);
      //   }
      // }

       cv::Mat ViewMat;
       cv::hconcat(left_cam, right_cam, ViewMat);
       cv::cvtColor(ViewMat, ViewMat, cv::COLOR_GRAY2RGB);
       // ViewMat+=mask;
       for (int i = 0; i < left_feats.size(); i++) {
         cv::circle(ViewMat, left_feats[i], 1, cv::Scalar(0, 0, 255));
       }

      std::vector<uint8_t> status;
      std::vector<float> err;
     


      cv::calcOpticalFlowPyrLK(left_cam, right_cam, left_feats, right_feats,
                               status, err, cv::Size(21,21), 4);

      

      std::cout<<"calcOpticalFlowPyrLK"<<std::endl;                       
      double fx = K.at<double>(0, 0);
      double fy = K.at<double>(1, 1);
      double cx = K.at<double>(0, 2);
      double cy = K.at<double>(1, 2);

      int count = std::count(status.begin(), status.end(), 1);
      std::default_random_engine dre;
      std::uniform_int_distribution<int> uinifor(0,255);
      for (int i = 0; i < right_feats.size(); i++) {
        
        if (status[i]) {
           
          cv::line(ViewMat,  left_feats[i],
                   right_feats[i] + cv::Point2f( left_cam.cols,0),
                   cv::Scalar(uinifor(dre), 0,uinifor(dre)));
          cv::circle(ViewMat,
                     right_feats[i] + cv::Point2f( left_cam.cols,0),
                     2, cv::Scalar(0, 255, 0));
        }
      }
      cv::imshow("ViewMatMask",ViewMat);
      cv::waitKey(10);  
      track_3dpoints.resize(count);
      track_2dpoints.resize(count);
      std::vector<cv::Point2f> feats_tracked(count);
      int j = 0;
      for (int i = 0; i < left_feats.size(); i++) {
        if (status[i]) {
          cv::Point3f point3f;
          cv::Point2f dp = left_feats[i] - right_feats[i];
          std::cout<<"dp="<<dp<<std::endl;
          if (dp.x > min_disparity && fabs(dp.y) < max_epipolar) {
            point3f.z = fx * base_line / dp.x;
            std::cout<<point3f.z<<std::endl;
            point3f.x = (left_feats[i].x - cx) / fx * point3f.z;
            point3f.y = (left_feats[i].y - cy) / fy * point3f.z;

          } else {
            point3f.z = 0;
            point3f.x = 0;
            point3f.y = 0;
          }
          track_3dpoints[j] = point3f;
          track_2dpoints[j] = left_feats[i];
          feats_tracked[j] = right_feats[i];
          j++;
        }
      }
      track_3dpoints.resize(j);
      track_2dpoints.resize(j);
      feats_tracked.resize(j);
      RemoveOutliner(track_2dpoints, feats_tracked, track_3dpoints);
      left_cam.copyTo(key_frame);
      k_pose_ = pose_;
      std::cout<<"k_pose_ = "<< k_pose_<<std::endl;
       
    }
    void Visualize(const cv::Mat& img,
                   const std::vector<cv::Point2f>& cur_feats,
                   std::vector<cv::Point2f> pre_feats, std::vector<uint8_t>& status) {
      cv::Mat img_feat;
      cv::cvtColor(img, img_feat,  cv::COLOR_GRAY2RGB);
      for(int i =0;i<cur_feats.size();i++){
        if(status[i]){
          cv::line(img_feat,cur_feats[i],pre_feats[i],cv::Scalar(0,0,255));
          //cv::circle(img_feat,pre_feats[i],2,cv::Scalar(255,0,0));
          cv::circle(img_feat,cur_feats[i],1,cv::Scalar(0,255,0));
        }
      }
      img_feat.copyTo(feats_img);
    }
    void RemoveOutliner(std::vector<cv::Point2f>& pre,
                        std::vector<cv::Point2f>& curr,
                        std::vector<cv::Point3f>& points) {
      std::vector<uint8_t> status;
      cv::Mat F =
          cv::findFundamentalMat(pre, curr, cv::FM_RANSAC, 1.0, 0.99, status);
      int j = 0;
      for (int i = 0; i < status.size(); i++) {
        if (status[i] && (i != j)) {
          pre[j] = pre[i];
          curr[j] = curr[i];
          points[j] = points[i];
          j++;
        }
      }
      pre.resize(j);
      curr.resize(j);
      points.resize(j);
   }
   bool TrackKeyFrame(const cv::Mat &key_img,const cv::Mat &curr_img)
   {
      std::vector<uint8_t> status;
      std::vector<float> err;
      std::vector<cv::Point2f> feats_curr;
      std::cout<<"TrackKeyFrame"<<std::endl;
      cv::calcOpticalFlowPyrLK(key_img, curr_img, track_2dpoints, feats_curr,
                               status, err,cv::Size(21,21),7);

      std::cout<<"track_2dpoints.size()"<<track_2dpoints.size()<<std::endl;
      std::cout<<"TrackKeyFrame1"<<std::endl;

      int count = std::count(status.begin(), status.end(), 1);

      std::vector<cv::Point3f> point3ds(count);
      std::vector<cv::Point2f> points(count);
      std::vector<cv::Point2f> points_curr(count);

      int j = 0;
      for (auto i = 0; i < status.size(); i++) {
        if (status[i]) {
          point3ds[j] = track_3dpoints[i];
          points_curr[j] = feats_curr[i];
          points[j] = track_2dpoints[i];
          j++;
        }
      }
    RemoveOutliner(points, points_curr, point3ds);
    std::cout<<"points_curr.size()"<<points_curr.size()<<std::endl;
    cv::Mat rvec,tvec;
  //  std::vector<uint8_t>  inliers;
    cv::Mat inliers;
    bool ret = cv::solvePnPRansac(point3ds, points_curr, K, dist_coeffs, rvec,
                                  tvec, false, 100, 10.0, 0.99, inliers,
                                  cv::SOLVEPNP_ITERATIVE);
    cv::Mat R;
    std::cout<<"inliers.size()="<<inliers.rows<<std::endl;
    status =  std::vector<uint8_t>(j,0);
    for (int i = 0; i < inliers.rows; i++) {
      int n = inliers.at<int>(i);
      status[n] = 1;
    }
    if (ret) {
      Visualize(curr_img,points_curr,points,status);       
      cv::Rodrigues(rvec, R);
      Eigen::Vector3d dt;
      Eigen::Matrix3d dr;
      
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          dr(i, j) = R.at<double>(i, j);
        }
        dt[i] = tvec.at<double>(i);
      }
      
      dt = -dr.transpose() * dt;
      std::cout<<"dt = "<<dt<<std::endl;
      std::cout<<"dr = "<<dr.transpose()<<std::endl;
      pose_.block<3,1>(0,3) =  k_pose_.block<3,1>(0,3) + k_pose_.block<3,3>(0,0)*dt;
      pose_.block<3,3>(0,0) = k_pose_.block<3,3>(0,0)*dr.transpose();
      std::cout<<"pose_= "<<pose_<<std::endl;
    }


    int inlier_count = std::count(status.begin(), status.end(), 1);
    std::cout<<"inlier_count = "<<inlier_count<<std::endl;
    return (inlier_count > min_feat_cnt);
   }
   const cv::Mat K =
       (cv::Mat_<double>(3, 3) << 385.0450439453125, 0.0, 323.1961975097656,
        0.0, 385.0450439453125, 244.11233520507812, 0.0, 0.0, 1.0);

   const  cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64FC1);
   const int max_feats_size = 1000;
   const int  min_feat_dist =10;
   const int min_disparity = 3;
   const int max_epipolar = 2;
   const  int min_feat_cnt = 50;
   const double base_line =  0.05;
   std::vector<cv::Point2f> track_2dpoints;
   std::vector<cv::Point3f> track_3dpoints;
   cv::Mat key_frame;
   Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
   Eigen::Matrix4d k_pose_= Eigen::Matrix4d::Identity();
   int states = 0;
   cv::Mat feats_img;


   VTrack  tracker_;

      
};





#endif