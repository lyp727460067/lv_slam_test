#include "match2d3d.h"
#include "Eigen/Geometry"

bool Match2d3d::InBound(const Eigen::Vector2f &img_point)
{
  if (img_point.x() < option_.imag_bound_.min().x() ||
      img_point.x() > option_.imag_bound_.max().x() ||
      img_point.y() > option_.imag_bound_.max().y() ||
      img_point.y() > option_.imag_bound_.max().y()

  ) {
    return false;
  }
  return true;
}
std::map<int, int> Match2d3d::MatchByProject(const Pose3d &init_pose,
                                             const MapPointsData &map_points,
                                             const ImagData &image_data,
                                             const bool &direction

)

{
  for(auto map_point:map_points.points){
    Eigen::Vector3f re_point = init_pose * map_point.second;
    if (re_point.z() < 0) continue;
    Eigen::Vector3f pre_point = option_.cam_instric * re_point;
    pre_point = pre_point/pre_point.z();
    if(InBound(pre_point.head<2>()))   {
      continue;
    }
    

    
  }

}