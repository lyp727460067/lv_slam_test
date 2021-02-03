#ifndef _PL_ICP_
#define _PL_ICP_

#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "icp_implementation.h"
#include "ceres/ceres.h"
#include "Eigen/Geometry"

#include "pcl/kdtree/kdtree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

class PointLineCostFunction {
 public:
  PointLineCostFunction(const Eigen::Vector3f p, const Eigen::Vector3f near_p1,
                        const Eigen::Vector3d near_p2)
      : p_(p), near_p1_(near_p1), near_p2_(near_p2) {}
  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {}
  static ceres::CostFunction* Creat(const Eigen::Vector3f p,
                                    const Eigen::Vector3f near_p1,
                                    const Eigen::Vector3d near_p2) {
    return new ceres::AutoDiffCostFunction<PointLineCostFunction, 3, 7>(
        new PointLineCostFunction(p, near_p1, near_p2));
  }

 private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f p_;
  Eigen::Vector3f near_p1_;
  Eigen::Vector3f near_p2_;
};
class  PlIcp:public IcpInterface
{
  public:
   bool Match(const Option& option, const Eigen::Matrix4f& init_pose,
              std::vector<Eigen::Vector3f> source_points,
              std::vector<Eigen::Vector3f> target_points,
              Eigen::Matrix4f& pose_estimate) {
                 
     pcl::PointCloud<PointType>::Ptr pcl_input_points =
         VectorPointToPclPointCloud(source_points);
     pcl::PointCloud<PointType>::Ptr pcl_targe_points =
         VectorPointToPclPointCloud(target_points);
    Eigen::Vector3f t;
    Eigen::Quaternion q;
    
    
     pcl::KdTreeFLANN<PointType> kd_tree;
     kd_tree.setInputCloud(pcl_targe_points);
     ceres::
     ceres::Problem problem;
     for (int i = 0; i < target_points.size(); i++) {
       PointType search_point = (*pcl_targe_points)[i];
       int k = 2;
       std::vector<int> index(k, -1);
       std::vector<float> distance(k);
       if (kd_tree.nearestKSearch(search_point, 2, index, distance) == k) {
         Eigen::Vector3f p = PclPointToEigenVector(search_point);
         Eigen::Vector3f near_p1 =
             PclPointToEigenVector(pcl_input_points->points[index[0]]);
         Eigen::Vector3f near_p2 =
             PclPointToEigenVector(pcl_targe_points->points[index[1]]);
         problem.AddResidualBlock(PointLineCostFunction::Creat(p, p1, p2), NULL,
                                  &t, &q);
       }
     }
   }

  private:
   using PointType = pcl::PointXYZ;
   pcl::PointCloud<PointType>::Ptr VectorPointToPclPointCloud(
       const std::vector<Eigen::Vector3f> points) {
     pcl::PointCloud<PointType>::Ptr point_cloude =
         new pcl::PointCloud<PointType>();
     for (auto point : points) {
       PointType pcl_point;
       pcl_point.x = point.x();
       pcl_point.y = point.y();
       pcl_point.z = point.z();
       point_cloude->push_back(pcl_point);
     }
     return point_cloude;
   }
   Eigen::Vector3f PclPointToEigenVector(const PointType& point) {
     Eigen::Vector3f vector3f;
     vector3f.x() = point.x;
     vector3f.y() = point.y;
     vector3f.z() = point.z;
     return vector3f;
   }
};







#endif