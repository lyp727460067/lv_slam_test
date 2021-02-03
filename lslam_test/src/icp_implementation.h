#ifndef _ICP_IMPLEMENTATION_H
#define   _ICP_IMPLEMENTATION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <limits>
#include <tuple>
#include <vector>

#include "Eigen/Core"
#include "glog/logging.h"
//#include "ceres/ceres.h"
class  IcpInterface
{
  public:
  class  Option
  {
    int inter_num =0;
    double err_tolerance  = 0.1;
  };
  virtual bool Match(const Option& option, const Eigen::Matrix4f& init_pose,
                     std::vector<Eigen::Vector3f> source_points,
                     std::vector<Eigen::Vector3f> target_points,
                     Eigen::Matrix4f& pose_estimate) = 0;
};

class  OriginalIcp:public IcpInterface
{
  public:
   bool Match(const Option& option, const Eigen::Matrix4f& init_pose,
              std::vector<Eigen::Vector3f> source_points,
              std::vector<Eigen::Vector3f> target_points,
              Eigen::Matrix4f& pose_estimate) {
     bool reverse = false;

     Eigen::MatrixXf source_temp = Eigen::MatrixXf::Zero(source_points.size(), 3);
     Eigen::MatrixXf source = Eigen::MatrixXf::Zero(source_points.size(), 3);
     Eigen::MatrixXf target = Eigen::MatrixXf::Zero(target_points.size(), 3);

     for (int i = 0; i < source_points.size(); i++) {
       source.row(i) = source_points[i];
     }
     for (int i = 0; i <target_points.size(); i++) {
       target.row(i) = target_points[i];
     }
     source_temp = source;
     Eigen::Matrix4f  T =  Eigen::Matrix4f::Identity();
     for (int inter = 0; inter < 10; inter++) {
       for(int i =0;i<source.rows();i++){
          source_temp.row(i) =  (T.block<3,3>(0,0)*source_temp.row(i).transpose()+T.block<3,1>(0,3));
       }
       LOG(INFO) << "start inter with  num " << inter;
     // std::cout<<source_temp<<target<<std::endl;
       Eigen::MatrixX4f Tst =GetPairPointsTransform({source_temp, target});
       T.block<3,3>(0,0) =  Tst.block<3,3>(0,0);
       T.block<3,1>(0,3)  = Tst.block<3,1>(0,3);
       
     }
     Eigen::MatrixX4f Tst = GetPairPointsTransform({source, source_temp});
     T.block<3, 3>(0, 0) = Tst.block<3, 3>(0, 0);
     T.block<3, 1>(0, 3) = Tst.block<3, 1>(0, 3);
     pose_estimate = T;
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
   Eigen::MatrixXf MinusMatrixMean(Eigen::MatrixXf m, Eigen::Vector3f mean) {
     Eigen::MatrixXf M(m.rows(),m.cols());
     for (int i = 0; i < m.rows(); i++) {
       M.row(i) =m.row(i)- mean.transpose();
     }
     return M;
   }
   std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> GetMinDistancMatchPonit(
       const std::tuple<Eigen::MatrixXf, Eigen::MatrixXf>& points) {
     Eigen::MatrixXf source_points;
     Eigen::MatrixXf target_points;
     std::tie(source_points, target_points) = points;   
     Eigen::MatrixXf match_target_points(source_points.rows(),
                                         source_points.cols());

     for (int i = 0; i < source_points.rows(); i++) {
       Eigen::Vector3f source_point = source_points.row(i);
       double pair_points_distance = std::numeric_limits<double>::max();
       for (int j = 0; j < target_points.rows(); j++) {
         double distance = (source_points.row(i) - target_points.row(j)).norm();
         if (distance <= pair_points_distance) {
           pair_points_distance = distance;
           match_target_points.row(i) = target_points.row(j);
         }
       }
     }
     return {source_points, match_target_points};
   }
   Eigen::Matrix4f GetPairPointsTransform(
      const std::tuple<Eigen::MatrixXf, Eigen::MatrixXf>& points) {
     Eigen::MatrixXf source_points;
     Eigen::MatrixXf target_points;
     std::tie( source_points, target_points)  = points;
     Eigen::MatrixXf match_target_points(source_points.rows(),
                                         source_points.cols());

     auto tar_mean = GetMeanVector(target_points);
     target_points = MinusMatrixMean(target_points, tar_mean);
     auto sor_mean = GetMeanVector(source_points);
     source_points = MinusMatrixMean(source_points, sor_mean);
     std::tie(source_points, target_points) = GetMinDistancMatchPonit({source_points, target_points});

     Eigen::Matrix3f M = source_points.transpose() * target_points;
     Eigen::JacobiSVD<Eigen::MatrixXf> svd(
         M, Eigen::ComputeThinU | Eigen::ComputeThinV);
     Eigen::Matrix3f U, V;
     U = svd.matrixU();
     V = svd.matrixV();
     auto R = V * U.transpose();
     auto t = tar_mean - R * sor_mean;
     Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
     T.block<3, 3>(0, 0) = R;
     T.block<3, 1>(0, 3) = t;
     return T;
   }
};

class  OpitmizeIcp :public IcpInterface
{
  public:
   OpitmizeIcp() {

  }

   bool Match(const Option& option, const Eigen::Matrix4f& init_pose,
              std::vector<Eigen::Vector3f> source_points,
              std::vector<Eigen::Vector3f> target_points,
              Eigen::Matrix4f& pose_estimate) {}
};


#endif