

#ifndef  MATCH2D3D_H
#define MATCH2D3D_H
#include <map>

#include "Eigen/Core"
#include "key_frame.h"
#include "opencv2/core.hpp"
#include "Eigen/Geometry"
class Pose3d
{
public:
  Eigen::Quaternionf& Rotaion(){
    return rotation;
  };
   Eigen::Vector3f& Translation() {
    return translation;
  };
  Eigen::Vector3f operator*(const Eigen::Vector3f & rhs){
    return rotation*rhs + translation;
  };
  Eigen::Vector3f operator*(const Eigen::Vector3f & rhs)const{
    return rotation*rhs + translation;
  };



private:
  Eigen::Quaternionf rotation;
  Eigen::Vector3f translation;
};

Eigen::Matrix3f operator*( const Eigen::Matrix3f &rhs,const Pose3d &lhs) {}
struct PointWithId
{


};


struct PointType
{
	

};

struct Match2d3dOption
{
	Eigen::Matrix3f cam_instric;
  Eigen::AlignedBox2f imag_bound_;

};
class Match2d3d {
 public:
  Match2d3d(const Match2d3dOption &option) {}

	private:
  std::map<int, int> MatchByProject(const Pose3d &init_pose,
                                    const MapPointsData &map_points,
                                    const ImagData &image_data,
                                    const bool &direction){

  };
  bool InBound(const Eigen::Vector2f &point);
  
  Match2d3dOption option_;
};

#endif