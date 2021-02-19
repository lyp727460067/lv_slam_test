#ifndef _STEREO_TEST_H
#define _STEREO_TEST_H
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv.hpp"
#include <vector>
#include "Eigen/Eigen"

class  StereoTrack
{
  public:
   
   bool Track(cv::Mat left_cam, cv::Mat right_cam) {
     if(states==0){
       Init(left_cam,right_cam);

     }

   }

   private:
    void Init(cv::Mat left_cam, cv::Mat right_cam) {
      std::vector<cv::KeyPoint> left_keypoints;
      std::vector<cv::Point2f> left_feats, right_feats;

      int thresh = 10;
      cv::Mat mask = cv::Mat(left_cam.size(), CV_8UC1, cv::Scalar(255));
      cv::Ptr<cv::FastFeatureDetector> detector =
          cv::FastFeatureDetector::create(thresh);
      detector->detect(left_cam, left_keypoints);
      std::sort(left_keypoints.begin(), left_keypoints.end(),
                [](const cv::KeyPoint& p1, const cv::KeyPoint& p2) {
                  return p1.response() > p2.response();
                });
      for (cv::KeyPoint keypoint : left_keypoints) {
        if (left_feats.size() < max_feats_size &&
            mask.at<uint8_t>(keypoint.pt.y, keypoint.pt.x)) {
          left_feats.push_back(keypoint.pt);
          cv::circle(mask, keypoint.pt, min_feat_dist, cv::Scalar(0),
                     cv::FILLED);
        }
      }

      std::vector<uint8_t> status;
      std::vector<float> err;
      cv::calcOpticalFlowPyrLK(left_cam, right_cam, left_feats, right_feats,
                               status, err);
      double fx = K.at<double>(0, 0);
      double fy = K.at<double>(1, 1);
      double cx = K.at<double>(0, 2);
      double cy = K.at<double>(1, 2);
      int count = std::cout(status.begin(), status.end(), 1);
      track_points.resize(count);
      track_features.resize(count);
      std::vector<cv::Point2f> feats_tracked(count);
      int j = 0;
      for (int i = 0; i < left_feats.size(); i++) {
        if (status[i]) {
          cv::Point3f point3f;
          cv::Point2f dp = left_feats[i] - right_feats[i];
          if (fabs(dp.x) > min_disparity && fabs(dp.y) < max_epipolar) {
            point3f.z = fx * base_line / dp.x;
            point3f.x = (left_feats[i].x - cx) / fx * point3f.z;
            point3f.y = (left_feats[i].y - cy) / fy * point3f.z;

          } else {
            point3f.z = 0;
            point3f.x = 0;
            point3f.y = 0;
          }
          track_points[i] = point3f;
          track_features[i] = left_feats[i];
          feats_tracked[j] = right_feats[i];
          j++;
        }
      }
      RemoveOutliner(left_feats, right_feats, track_points);
      left_cam.copyTo(key_frame);
      k_pose_ = pose_;
    }

   void RemoveOutliner(std::vector<cv::Point2f>& pre,
                       std::vector<cv::Point2f>& curr,
                       std::vector<cv::Point3f>& points) {
     std::vector<uint8_t> status;
     cv::Mat F =
         cv::findFundamentalMat(pre, curr, CV_FM_RANSAC, 1.0, 0.99, status);
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
   bool TrackKeyFrame(cv::Mat key_img,cv::Mat curr_img)
   {
    std::vector<uint8_t> status;
      std::vector<float> err;
      std::vector<cv::Point2f> feats_curr;
      cv::calcOpticalFlowPyrLK(key_img, curr_img, track_features, feats_curr,
                               status, err);

      int count = std::count(status.begin(), status.end(), 1);

      std::vector<cv::Point3f> point3ds(count);
      std::vector<cv::Point2f> points(count);
      std::vector<cv::Point2f> points_curr(count);

      int j = 0;
      for (auto i = 0; i < status.size(); i++) {
        if (status[i]) {
          point3ds[j] = track_points[i];
          points_curr[j] = feats_curr[i];
          points[j] = track_features[i];
          j++;
        }
      }
    RemoveOutliner(points, points_curr, point3ds);

    cv::Mat rvec,tvec;
    std::vector<uint8_t>  inliers;
    bool ret = cv::solvePnPRansac(point3ds, points_curr, K, dist_coeffs, rvec,
                                  tvec, false, 30, 6.0, 0.99, inliers,
                                  cv::SOLVEPNP_ITERATIVE);
    cv::Mat R;
    if (ret) {
      cv::Rodrigues(rvec, R);
      Eigen::Vector3d dt;
      Eigen::Matrix3d dr;
      
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          dr(i, j) = R.at<double>(i, j);
        }
        dt[i] = rvec.at<double>(i);
      }
      dt = -dr.transpose() * dt;
      pose_.block<3,1>(0,3) =  k_pose_.block<3,1>(0,3) + k_pose_.block<3,3>(0,0)*dt;
      pose_.block<3,3>(0,0) = k_pose_.block<3,3>(0,0)*dr.transpose();
    }
    int inlier_count = std::count(inliers.begin(), inliers.end(), 1);
    return inlier_count > min_feat_cnt;
   }
   const cv::Mat K =
       (cv::Mat_<double>(3, 3) << 385.0450439453125, 0.0, 323.1961975097656,
        0.0, 385.0450439453125, 244.11233520507812, 0.0, 0.0, 1.0);

   const  cv::Mat dist_coeffs = cv::Mat(5,1,CV_64F,cv::Scalar(0));
   const int max_feats_size = 1000;
   const int  min_feat_dist = 30;
   const int min_disparity = 2;
   const int max_epipolar = 5;
   const  int min_feat_cnt = 50;
   const double base_line =  0.1;
   std::vector<cv::Point2f> track_features;
   std::vector<cv::Point3f> track_points;
   cv::Mat key_frame;
   Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
   Eigen::Matrix4d k_pose_;
   int states = 0;
  
      
};





#endif