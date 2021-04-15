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
struct KeyPoint {
  cv::Point2f point;
  Eigen::Vector2d velocity;
  double depth = -1;
};
class Frame {
 public:
  std::map<int,KeyPoint> keyPoints_;
  Eigen::Vector3d pose_t =Eigen::Vector3d::Identity();
  Eigen::Quaterniond  pose_q = Eigen::Quaterniond::Identity();


};


;

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
    ReProjectionErr( float x, float y,float rx,float ry,float rz):x_(x),y_(y),rx_(rx),ry_(ry),rz_(rz){
      float fx = (K.at<float>(0, 0));
      float cx = (K.at<float>(0, 2));
      float cy = (K.at<float>(1, 2));
      rx_ = ((rx_* rz_)-cx)  / fx;
      ry_ = ((ry_* rz_)-cy)  / fx;
    }

  template <typename T>
  bool operator()(const T* t1_, const T* q1_, const T* t2_, const T* q2_,
                   T* residul) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t1(t1_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t2(t2_);
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_);
    Eigen::Map<const Eigen::Quaternion<T>> q2(q2_);

    Eigen::Quaternion<T> relative_q = q2.inverse() * q1;
    Eigen::Matrix<T, 3, 1> relative_t = q2.inverse().toRotationMatrix() * t1 -
                                        q2.inverse().toRotationMatrix() * t2;

    Eigen::Matrix<T, 3, 1> rel_p{T(rx_), T(ry_), T(rz_)};
    Eigen::Matrix<T, 3, 1> project_p =
        relative_q.toRotationMatrix() * rel_p + relative_t;
    T fx = T(K.at<float>(0, 0));
    T cx = T(K.at<float>(0, 2));
    T cy = T(K.at<float>(1, 2));
    T u = (project_p[0] * fx + cx) / project_p[2];
    T v = (project_p[1] * fx + cy) / project_p[2];

    residul[0] = (T(x_) - u);
    residul[1] =(T(y_) - v);

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
  Eigen::Matrix4d pose_end_;


};

class PoseSlideWindow
{
  public:
  SliedeWindowResult  Insert(const Frame& frame)
  {

    double sum_parallax ;
    for(auto point:frame.keyPoints_){
      sum_parallax+=(point.second.velocity).cwiseAbs().norm();
    }
    double  ave_parellax =sum_parallax/frame.keyPoints_.size();
    std::cout<<"ave_parellax"<<ave_parellax<<std::endl;
    if (frames_.size() == window_size) {
     // if (ave_parellax < 10) {
     //   DeleteFrame((--frames_.end())->first);
    //  } else {
        DeleteFrame(frames_.begin()->first);
   //  }
    }
    InsertFrame(frame);
    Opimizetion();

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
   
    pose.block<3,3>(0,0)  =  (--frames_.end())->second->pose_q.toRotationMatrix();
    pose.block<3,1>(0,3)  =  (--frames_.end())->second->pose_t;
    return {pose};
   
  };

  
  std::vector<Eigen::Matrix4f> GetSlidesPose(){
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    std::vector<Eigen::Matrix4f> pose_t;
    for(auto const & frame:frames_){
      std::cout<<frame.second->keyPoints_[0].point<<std::endl;

      auto q  =  frame.second->pose_q;
      auto t  =  frame.second->pose_t;
      pose.block<3, 3>(0, 0) = q.toRotationMatrix().template cast<float>();
      pose.block<3, 1>(0, 3) = t.template cast<float>();
      pose_t.push_back(pose);
    }
    return pose_t;
  }

  private:
   void Opimizetion() {
     if(frames_.size()<window_size)return ;
     std::cout<<frames_.size()<<std::endl;
     ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
     ceres::LocalParameterization* quaternion_local =
         new ceres::EigenQuaternionParameterization;
            for(auto corres:key_point_corresponding_){
      std::cout<<corres.first<<" "<<corres.second<<std::endl;
    }


     for (auto iter = key_point_corresponding_.begin();
          iter != key_point_corresponding_.end();
          iter = key_point_corresponding_.upper_bound(iter->first)) {
       int count = key_point_corresponding_.count(iter->first);
       std::cout<<iter->first<<std::endl;
       std::cout<<count<<std::endl;
       std::cout<<"i"<<std::endl;
       if (count < 2) continue;
       std::pair<std::multimap<int, int>::iterator,
                 std::multimap<int, int>::iterator>
           range_iter = key_point_corresponding_.equal_range(iter->first);
       for (auto it = std::next(range_iter.first); it != range_iter.second; it++) {
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
         if(rz <0 )continue;
         std::cout<<"test"<<std::endl;
        // std::cout<<"first frame id="<<range_iter.first->second<<"sencode frame id ="<<it->second<<"\n"
        // <<"with key point id = "<<it->first <<" rz= "<<rz<< "\n"
        // <<"  u = "<<u<< "  v  = "<<v<<std::endl;

         problem.AddResidualBlock(
             ReProjectionErr::Creat(u, v, rx, ry, rz), loss_function,
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
     problem.SetParameterBlockConstant(
         frames_.begin()->second->pose_q.coeffs().data());
     problem.SetParameterBlockConstant(frames_.begin()->second->pose_t.data());
     options.minimizer_progress_to_stdout = false;
     options.linear_solver_type = ceres::DENSE_SCHUR;
     options.num_threads = 12;
     options.trust_region_strategy_type = ceres::DOGLEG;
     ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //delete loss_function;
    //delete quaternion_local;

   }

  int window_size  = 10;
  void InsertFrame(const Frame&frame){
    Frame * temp_fram  = new Frame();
    *temp_fram = frame;
    frames_.insert({frame_id,temp_fram});
    for(auto featur_id:frame.keyPoints_){
        key_point_corresponding_.insert({featur_id.first,frame_id});
    }
   frame_id++;
  }
  void DeleteFrame(int frame_id){
    std::cout<<frame_id<<std::endl;
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
    frames_[frame_id]  = nullptr;    
    delete  frames_[frame_id];
    frames_.erase(frame_id);
    // std::cout<<"delete succudes"<<std::endl;
    // for(auto corres:key_point_corresponding_){
    //   std::cout<<corres.second<<std::endl;
    // }


  }



   int frame_id = 0; 
  std::map<int,Frame*> frames_;
  std::multimap<int,int>  key_point_corresponding_;//feature id  frame id;


};




#endif
