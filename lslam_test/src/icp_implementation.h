#ifndef _ICP_IMPLEMENTATION_H
#endif  _ICP_IMPLEMENTATION_H
#include "Eigen/Core"
#include <vector>
#include <limits>
#include <Eigen/SVD>  
#include <Eigen/Dense>   
#include <algorithm>
class  icp_interface
{

};

class  OriginalIcp:public icp_interface
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
       tar.resize(source.size());
     }
    

     for (int inter = 0; inter < 10; inter++) {
       for (int i = 0; i < source.size(); i++) {
         Eigen::Vector3f source_point = source[i];
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

       Eigen::Matrix3f M = sor.reverse() * tar;
       Eigen::JacobiSVD<Eigen::MatrixXf> svd(
           M, Eigen::ComputeThinU | Eigen::ComputeThinV);
       Eigen::Matrix3f U, V, R;
       U = svd.matrixU();
       V = svd.matrixV();
       R = U * V.transpose();
    }
   }


   

  private:
   Eigen::Matrix3f GetMeanVector(const Eigen::MatrixXf& m) {
     Eigen::Matrix3f vector_sum = {0, 0, 0};
     for (int i = 0; i < m.rows(); i++) {
       vector_sum += m.block<1, 3>(i, 0);
     }
     return vector_sum / m.rows();
   }
   void GetMatrixOffset(Eigen::MatrixXf& m, Eigen::Matrix3f offset) {
     for (int i = 0; i < m.rows(); i++) {
       m.block<1, 3>(i, 0) -= offset;
     }
   }
};



#endif