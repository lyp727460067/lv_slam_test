#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <memory>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <pcl/features/normal_3d.h>
#include "glog/logging.h"
namespace{
using PointType = pcl::PointXYZ;
pcl::PointCloud<PointType>::Ptr map_point(
    new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr curr_point(
      new pcl::PointCloud<PointType>());
std::condition_variable condition;
std::mutex new_point_mutex;
}


bool new_cloud  = false;
bool map_finish = false;



void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr msg) {
  {
    std::lock_guard<std::mutex> lock(new_point_mutex);
    pcl::fromROSMsg(*msg, *curr_point);
    new_cloud = true;
  }
  condition.notify_all();
  ROS_INFO("new scan comming");
 }

 Eigen::Matrix4f pose_expet= Eigen::Matrix4f::Identity();
 Eigen::Matrix4f delta_pose = Eigen::Matrix4f::Identity();
 Eigen::Matrix4f last_pose= Eigen::Matrix4f::Identity();

double leaf_size  =  10;
class MatcherInterface {
 public:
  class Option {};
  virtual bool ScanMatch(Eigen::Matrix4f init_pose,
                     pcl::PointCloud<PointType>::Ptr Source,
                     pcl::PointCloud<PointType>::Ptr Target,
                     const Option option, double score,
                     Eigen::Matrix4f& expect_pose) = 0;

   std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> CaculatePointCloudNormal(
      std::vector<pcl::PointCloud<PointType>::Ptr> sources) {
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> point_with_normal;
    for (auto source : sources) {
      pcl::NormalEstimation<PointType, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals(
          new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<PointType>::Ptr tree(
          new pcl::search::KdTree<PointType>);

      tree->setInputCloud(source);
      n.setInputCloud(source);
      n.setSearchMethod(tree);
      n.setKSearch(20);
      n.compute(*normals);
 
      pcl::PointCloud<pcl::PointNormal>::Ptr init_trans_cloud_normals(
          new pcl::PointCloud<pcl::PointNormal>);
      pcl::concatenateFields(*source, *normals, *init_trans_cloud_normals);
      point_with_normal.push_back(init_trans_cloud_normals);

    }
    return point_with_normal;
  }

 ~MatcherInterface(){};
};
class IcpMatcher : public MatcherInterface {
 public:
  IcpMatcher() {
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(0.01);
    icp.setEuclideanFitnessEpsilon(0.01);
    icp.setRANSACIterations(50);
  }
  bool ScanMatch(Eigen::Matrix4f init_pose, pcl::PointCloud<PointType>::Ptr Source,
             pcl::PointCloud<PointType>::Ptr Target, const Option option,
             double score, Eigen::Matrix4f& expect_pose) {

    std::vector<pcl::PointCloud<PointType>::Ptr> caculate_normal_point_cloud;
    caculate_normal_point_cloud.push_back(Source);
    caculate_normal_point_cloud.push_back(Target);
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> PointCloudWithNormal =
        CaculatePointCloudNormal(caculate_normal_point_cloud);
    typedef pcl::registration::TransformationEstimationPointToPlaneLLS<
        pcl::PointNormal, pcl::PointNormal>
        PointToPlane;
    //typedef pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<
     //   pcl::PointNormal, pcl::PointNormal>
      //  SymmPointToPlane;

    PointToPlane::Ptr point_to_plane(new PointToPlane);
//    SymmPointToPlane::Ptr point_to_plane(new SymmPointToPlane);

    icp.setTransformationEstimation(point_to_plane);
    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*PointCloudWithNormal[0],*PointCloudWithNormal[0],indices);
    pcl::removeNaNNormalsFromPointCloud(*PointCloudWithNormal[1],*PointCloudWithNormal[1],indices);
    icp.setInputSource(PointCloudWithNormal[0]);
    icp.setInputTarget(PointCloudWithNormal[1]);
    pcl::PointCloud<pcl::PointNormal>::Ptr unused_result(
        new pcl::PointCloud<pcl::PointNormal>());
    icp.align(*unused_result, init_pose);
    if (icp.hasConverged()) {
      expect_pose = icp.getFinalTransformation();

      score = icp.getFitnessScore();
      return true;
    }
    return false;
  }

 private:
  pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
};




class NdtMatcher : public MatcherInterface {
 public:
  NdtMatcher() {
    ndt.setTransformationEpsilon(0.1);
    ndt.setStepSize(0.5);
    ndt.setResolution(5.0);
    ndt.setMaximumIterations(30);
  }
  bool ScanMatch(Eigen::Matrix4f init_pose, pcl::PointCloud<PointType>::Ptr Source,
             pcl::PointCloud<PointType>::Ptr Target, const Option option,
             double score, Eigen::Matrix4f& expect_pose) {

    ndt.setInputSource(Source);
    ndt.setInputTarget(Target);
    pcl::PointCloud<PointType>::Ptr unused_result(
        new pcl::PointCloud<PointType>());
    ndt.align(*unused_result, init_pose);
    if (ndt.hasConverged()) {
      expect_pose = ndt.getFinalTransformation();
      score = ndt.getFitnessScore();
      return true;
    }
    return false;
  }
 private:
  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
};
tf::TransformBroadcaster* tf_broadcaster;
void PubTf(Eigen::Matrix4f pose) {
  geometry_msgs::TransformStamped tf_trans;
  Eigen::Matrix3f ratation  = pose.block(0,0,3,3);
  Eigen::Quaternionf q(ratation);
  tf_trans.header.stamp = ros::Time::now();
  tf_trans.header.frame_id = "map";
  tf_trans.child_frame_id = "base_link";
  tf_trans.transform.translation.x = pose(0,3);
  tf_trans.transform.translation.y = pose(1,3);
  tf_trans.transform.translation.z = pose(2,3);
  tf_trans.transform.rotation.x = q.x();
  tf_trans.transform.rotation.y = q.y();
  tf_trans.transform.rotation.z = q.z();
  tf_trans.transform.rotation.w = q.w();
  tf_broadcaster->sendTransform(tf_trans);
}
void NdtProgress(void) {
  std::cout << "start NdtProgress" << std::endl;

  pcl::PointCloud<PointType>::Ptr pre_cloud(
      new pcl::PointCloud<PointType>);
  uint64_t delta_time;

  while (ros::ok) {
    std::unique_lock<std::mutex> lock(new_point_mutex);
    condition.wait(lock, []() { return new_cloud; });
    new_cloud = false;

    std::vector<int> indices;
    pcl::PointCloud<PointType>::Ptr filter_cloud1(
        new pcl::PointCloud<PointType>);
    std::cout<< "is_dense"<<curr_point->is_dense <<std::endl;
    pcl::removeNaNFromPointCloud(*curr_point, *curr_point, indices);
    ROS_INFO("INPUT_SIZE =  %d", curr_point->size());

    pcl::PointCloud<PointType>::Ptr filter_cloud(
        new pcl::PointCloud<PointType>);
    pcl::ApproximateVoxelGrid<PointType> filtered_cloud_votex;
    filtered_cloud_votex.setLeafSize(leaf_size, leaf_size, leaf_size);
    filtered_cloud_votex.setInputCloud(curr_point);
    filtered_cloud_votex.filter(*filter_cloud);
    // curr_point->clear();
    ROS_INFO("filter_cloud.size%d", filter_cloud->size());
    if (map_point->empty()) {
      ROS_INFO("first scan input");
      *map_point += *filter_cloud;
      *pre_cloud = *filter_cloud;
      ROS_INFO("map_point.size%d", map_point->size());
      continue;
    }
    std::unique_ptr<MatcherInterface> matcher = std::unique_ptr<IcpMatcher>(new IcpMatcher());
    double score = 0.0;
    if (matcher->ScanMatch(delta_pose, filter_cloud, pre_cloud,
                       MatcherInterface::Option(), score, delta_pose)) {
      std::cout << "\nICP has converged, score is " << score << std::endl;
      if (score < 0.1) {
        pre_cloud->clear();
        *pre_cloud = *filter_cloud;
        std::cout << "curr_pos" << delta_pose << std::endl;
        Eigen::Matrix4f curr_pos = last_pose * delta_pose;
        last_pose = curr_pos;
        std::cout << "last_pose" << last_pose << std::endl;
        map_point->clear();
        pcl::transformPointCloud(*curr_point, *map_point, curr_pos);
        map_finish = true;
        PubTf(curr_pos);
      }
    }
    curr_point->clear();
  }
}
std::vector<nav_msgs::Odometry> odom_datas_;
Eigen::Matrix4f ExtroplateOdom(uint64_t start_time,uint64_t end_time)
{


}
int main(int argc, char** argv) {
  ros::init(argc, argv, "lslam_test");
  tf_broadcaster = new tf::TransformBroadcaster;
  std::string scan_topic;
  std::string odom_topic;
  ros::NodeHandle private_node("~");
  std::string point2_topic;
  private_node.param<double>("leaf_size",leaf_size,10.0);
  private_node.param<std::string>("scan",scan_topic,"/scan");
  private_node.param<std::string>("odom",odom_topic,"/odom");
  private_node.param<std::string>("point2_topic",point2_topic,"/kitti/velo/pointcloud");
   
  ROS_INFO("\003[1;32--->\003[0m lsma_test");
  if (argc != 2) {
    ROS_INFO("please input bag file");
  }
  laser_geometry::LaserProjection projector_;
  ros::NodeHandle nh;
  std::string inputbag_name(argv[1]);
  tf::TransformListener tfListener_;
  tfListener_.setExtrapolationLimit(ros::Duration(0.1));

  std::thread tread_bag([&]() {
    rosbag::Bag in_bag;
    in_bag.open(inputbag_name, rosbag::bagmode::Read);
    rosbag::View view(in_bag);
    rosbag::View::const_iterator view_iterator = view.begin();
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2::ConstPtr cloud_ptr(&cloud);     
    for (auto view_iterator = view.begin(); view_iterator != view.end();
         view_iterator++) {
      rosbag::MessageInstance msg = *view_iterator;
      if (msg.isType<sensor_msgs::PointCloud2>()) {
        if (msg.getTopic() == point2_topic) {
          PointCloudCallBack(msg.instantiate<sensor_msgs::PointCloud2>());
        }
      }
      if (msg.isType<sensor_msgs::LaserScan>()) {
        if (msg.getTopic() == scan_topic) {
          sensor_msgs::LaserScan scan =
              *msg.instantiate<sensor_msgs::LaserScan>();
          // tfListener_.waitForTransform(
          // "base_link","laser",ros::Time(0),ros::Duration(1.0));;
        
          projector_.projectLaser(scan, cloud);
          PointCloudCallBack(cloud_ptr);
        }
      }
      if (msg.isType<nav_msgs::Odometry>()) {
        if (msg.getTopic() == odom_topic) {
          nav_msgs::Odometry odom = *msg.instantiate<nav_msgs::Odometry>();
          odom_datas_.push_back(odom);
          // tfListener_.waitForTransform(
          // "base_link","laser",ros::Time(0),ros::Duration(1.0));;
        }
      }
    }
  });
  // ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::PointCloud2>(
  //     "/kitti/velo/pointcloud", 1, &PointCloudCallBack);
  ros::Publisher map_pub =
      nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 2);

  std::thread ndt_thread(&NdtProgress);
  ros::Rate rate(10);
  int stamp = 0;
  std::cout<<"start main"<<std::endl; 
  while (ros::ok()) {
    if (map_finish) {
      sensor_msgs::PointCloud2 points2;     
      map_finish = false;
      pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
      *cloudOut+=*map_point;
      pcl::toROSMsg(*cloudOut, points2);
      points2.header.frame_id = "map";
      points2.header.stamp = ros::Time::now();
      map_pub.publish(points2);
      rate.sleep();
    }
    ros::spinOnce();
  }
  ros::shutdown();
}