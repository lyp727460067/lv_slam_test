#ifndef _PL_ICP_
#define _PL_ICP_


#include "ceres/ceres.h"
#include "icp_implementation.h"

class PointLineCostFunction
{
  public:
   PointLineCostFunction(const Eigen::Vector3f p, const Eigen::Vector3f near_p1,
                         const Eigen::Vector3d near_p2)
       : p_(p), near_p1_(near_p1), near_p2_(near_p2) {}
   template <typename T>
   bool operator()(const T* const pose, T* residuals) const {}

  private:
   Eigen::Vector3f p_;
   Eigen::Vector3f near_p1_;
   Eigen::Vector3f near_p2_;
};
class  PlIcp:public IcpInterface
{
  public:
   bool Match(const Option& option,const Eigen::Matrix4f& init_pose,
              std::vector<Eigen::Vector3f> source_points,
              std::vector<Eigen::Vector3f> target_points,
                  Eigen::Matrix4f& pose_estimate) {
  ceres::Problem problem;


}

  private:
 



};







#endif