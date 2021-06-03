#ifndef _TRANSFOR_H
#define _TRANSFOR_H
#include "Eigen/Core"
struct	 Rigid3d
{
	
	Eigen::Matrix3f rotation;
	Eigen::Vector3f translate;
};


#endif