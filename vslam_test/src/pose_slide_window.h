#ifndef _POSE_SLIDE_WINDOW_H
#define  _POSE_SLIDE_WINDOW_H
#include <map>
#include <memory>
#include <vector>


#include  <Eigen/Core> 

#include "ceres/autodiff_cost_function.h"
#include "ceres/ceres.h"
#include "key_frame.h"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

class Frame {
 public:
  std::map<int,KeyPoint> keyPoints_;
  Eigen::Vector3f pose_t;
  Eigen::Quaternionf  pose_q;


};


struct KeyPoint {
  cv::Point2f point;
  Eigen::Vector2f velocity;
  double depth = -1;
};

// class AngleLocalParameterization {
//   public:
//   template <typename T>
//   bool operator()(const T* theta_radians, const T* delta_theta_radians,
//                   T* theta_radians_plus_delta) const {
//     *theta_radians_plus_delta =
//         NormalizeAngle(*theta_radians + *delta_theta_radians);
//     return true;
//   }

//   static ceres::LocalParameterization* Create() {
//     return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
//                                                      1, 1>);
//   }
// };

struct ReProjectionErr {
 public:
    ReProjectionErr( float x, float y,float rx,float ry,float rz):x_(x),y_(y),rx_(rx),ry_(ry),rz_(rz){}

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, const T* t2_, const T* q2_,
                   T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t2(t2_);
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    Eigen::Map<const Eigen::Quaternion<T>> q2(q2_);

    T fx = T(K(0, 0));
    T cx = T(K(0, 2));
    T cy = T(k(1, 2));
    T z = T(rz_);
    T x = (T(rx_) - cx) * z / fx;
    T y = (T(ry_) - cy) * z / fx;

    Eigen::Quaternion<T> relative_q = q1.conjugate() * q2;
    Eigen::Matrix<T, 3, 1> relative_t = q1.conjugate().toRotationMatrix() * t2 -
                                        q1.conjugate().toRotationMatrix() * t1;

    Eigen::Matrix<T, 3, 1> p(x, y, z);
    Eigen::Matrix<T, 3, 1> project_p =
        relative_q.toRotationMatrix() * rel_p + relative_t;

    T u = (project_p[0] * fx + cx) / z;
    T v = (project_p[1] * fx + cy) / z;

    residul[0] = x_ - u;
    residul[1] = y_ v;

    return true;
  }
  static  ceres::CostFunction* Creat(float x,float y,float rx,float ry,float rz){
    return new ceres::AutoDiffCostFunction<ReProjectionErr,2,3,4,3,4>(
        new ReProjectionErr(x, y,rx,ry,rz));
  }
 private:
  const cv::Mat K =
      (cv::Mat_<float>(3, 3) << 385.0450439453125, 0.0, 323.1961975097656, 0.0,
       385.0450439453125, 244.11233520507812, 0.0, 0.0, 1.0);
  
  float x_;
  float y_;

  float rx_;
  float ry_;
  float rz_;
};


struct SliedeWindowResult
{



};

class PoseSlideWindow
{
  public:
  SliedeWindowResult  Insert(const Frame& frame)
  {

    Eigen::Vector2d  sum_parallax ;
    for(auto point:frame.keyPoints_){
      sum_parallax+=point.second.velocity;
    }
    Eigen::Vector2d ave_parellax =sum_parallax/frame.keyPoints_.size();
    if (frames_.size() == window_size) {
      if (ave_parellax[0] < 10) {
        DeleteFrame(frames_.end()->first);
      } else {
        DeleteFrame(frames_.begin()->first);
      }
    }
    InsertFrame(frame);
    Opimizetion();

  };

  


  private:
   void Opimizetion() {
     ceres::Problem problem;
     ceres::LocalParameterization* quaternion_local =
         new ceres::EigenQuaternionParameterization;
     ceres::LossFunction* loss_function = NULL;
     for (auto iter = key_point_corresponding_.begin();
          iter != key_point_corresponding_.end();
          iter = key_point_corresponding_.upper_bound(iter->first)) {
       int count = key_point_corresponding_.count(iter->first);
       if (count < 2) continue;
       std::pair<std::multimap<int, int>::iterator,
                 std::multimap<int, int>::iterator>
           range_iter = key_point_corresponding_.equal_range(iter->first);
       for (auto it = range_iter.first++; it != range_iter.second; it++) {
         float u = frames_[it->second]->keyPoints_[it->first].point.x;
         float v = frames_[it->second]->keyPoints_[it->first].point.y;
         float rx = frames_[range_iter.first->second]
                         ->keyPoints_[range_iter.first->first]
                         .point.x;
         float ry = frames_[range_iter.first->second]
                         ->keyPoints_[range_iter.first->first]
                         .point.y;
         float rz = frames_[range_iter.first->second]
                         ->keyPoints_[range_iter.first->first]
                         .depth;
         if(rz <0)continue;
         problem.AddResidualBlock(
             ReProjectionErr::Creat(u, v, rx, ry, rz), NULL,
             frames_[range_iter.first->second]->pose_t.data(),
             frames_[range_iter.first->second]->pose_q.coeffs().data(),
             frames_[it->second]->pose_t.data(),
             frames_[it->second]->pose_q.coeffs().data());

         problem.SetParameterization(
             frames_[range_iter.first->second]->pose_q.coeffs().data(),
             quaternion_local);
         problem.SetParameterization(
             frames_[it->second]->pose_q.coeffs().data(), quaternion_local);
       }
     }

     ceres::Solver::Options options;
     options.minimizer_progress_to_stdout = false;
     options.max_num_iterations = 20;
     options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
     ceres::Solver::Summary summary;
     ceres::Solve(options, &problem, &summary);
     problem.SetParameterBlockConstant(
         frames_.begin()->second->pose_q.coeffs().data());
     problem.SetParameterBlockConstant(frames_.begin()->second->pose_t.data());
   }

  int window_size  = 10;
  void InsertFrame(const Frame&frame){
    Frame * temp_fram  = new Frame();
    *temp_fram = frame;
    frames_.insert({frame_id++,temp_fram});
  }
  void DeleteFrame(int frame_id){
    for (auto point : frames_[frame_id]->keyPoints_) {
      int count = key_point_corresponding_.count(point.first);
      if (count != 0) {
        auto it = key_point_corresponding_.find(point.first);
        for (int i = 0; i < count; i++) {
          if (it->second == frame_id) {
            it = key_point_corresponding_.erase(it);
          } else {
            ++it;
          }
        }
      }
    }
    delete  frames_[frame_id];
    frames_[frame_id]  = nullptr;
    frames_.erase(frame_id);

  }



  static int frame_id; 
  std::map<int,Frame*> frames_;
  std::multimap<int,int>  key_point_corresponding_;//feature id  frame id;


};




#endif
