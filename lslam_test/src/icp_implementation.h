#ifndef _ICP_IMPLEMENTATION_H
#define   _ICP_IMPLEMENTATION_H
#include "Eigen/Core"
#include <vector>
#include <limits>
#include <Eigen/SVD>  
#include <Eigen/Dense>   
#include <algorithm>
#include  "glog/logging.h"
class  IcpInterface
{

};

class  OriginalIcp:public IcpInterface
{
  public:
   bool Match(Eigen::Matrix4d& init_pose, std::vector<Eigen::Vector3f> source,
              std::vector<Eigen::Vector3f> target) {
     bool reverse = false;
     Eigen::MatrixXf sor = Eigen::MatrixXf::Zero(source.size(), 3);
     Eigen::MatrixXf tar = Eigen::MatrixXf::Zero(target.size(), 3);

     if (target.size() < source.size()) {
       reverse = true;
       source.swap(target);
       tar.resize(source.size(), 3);
     }
     Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
     Eigen::Vector3f t{0, 0, 0};
     std::vector<Eigen::Vector3f> source1 = source;
     for (int inter = 0; inter < 10; inter++) {
       LOG(INFO) << "start inter with  num " << inter;
       for (int i = 0; i < source1.size(); i++) {
         Eigen::Vector3f source_point = source1[i];
         sor.block<1, 3>(i, 0) = source_point;
         tar.block<1, 3>(i, 0) = target[i];

         double tow_point_distance = std::numeric_limits<double>::max();
         for (int j = 0; j < target.size(); j++) {
           Eigen::Vector3f target_point = target[j];
           double distance = (target_point - source_point).norm();
           if (distance < tow_point_distance) {
             tow_point_distance = distance;
             tar.block<1, 3>(i, 0) = target[j];
           }
         }
       }
       auto tar_mean = GetMeanVector(tar);
       GetMatrixOffset(tar, tar_mean);
       auto sor_mean = GetMeanVector(sor);
       GetMatrixOffset(sor, sor_mean);

       Eigen::Matrix3f M = sor.transpose() * tar;
       Eigen::JacobiSVD<Eigen::MatrixXf> svd(
           M, Eigen::ComputeThinU | Eigen::ComputeThinV);
       Eigen::Matrix3f U, V;
       U = svd.matrixU();
       V = svd.matrixV();
       auto delata_r = V * U.transpose();
       R =    delata_r *R;
       auto diff_vector = tar_mean -  delata_r*sor_mean;   
       t += diff_vector ;          
       source1.clear();
       for (auto& sor : source) { 
         auto sor_temp = R * sor + t;
         source1.push_back(sor_temp);
       }

       std::cout << t << std::endl;       
    }
    init_pose.block<3,3>(0,0)  =  R.cast<double>();
    init_pose.block<3,1>(0,3) = t.transpose().cast<double>();

    return true;
   }


   

  private:
   Eigen::Vector3f  GetMeanVector(const Eigen::MatrixXf& m) {
     Eigen::Vector3f vector_sum = {0, 0, 0};
     for (int i = 0; i < m.rows(); i++) {
       vector_sum += m.block<1, 3>(i, 0).transpose();
     }

     return vector_sum / m.rows();
   }
   void GetMatrixOffset(Eigen::MatrixXf& m, Eigen::Vector3f offset) {
     for (int i = 0; i < m.rows(); i++) {
       m.block<1, 3>(i, 0) -= offset;
     }
   }
};

class  IpIcp :public IcpInterface
{





};





#endif