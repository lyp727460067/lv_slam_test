#include <string>
#include <thread>
#include <vector>
#include <tuple>

#include "image_transport/image_transport.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/CompressedImage.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "stereo_test.h"
#include <geometry_msgs/Point32.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "gflags/gflags.h"
#include "glog/logging.h"


StereoTrack StereoTrack_;



tf::TransformBroadcaster* tf_broadcaster;
ros::Publisher point_cloud_pub;
ros::Publisher feat_img_pub;
void PubPointCloud2(const std::vector<cv::Point3f>& points) {
  sensor_msgs::PointCloud point_cloud;
  sensor_msgs::PointCloud2 point_cloud2;
  for (auto point : points) {
    geometry_msgs::Point32 geo_point;
    geo_point.x = point.x;
    geo_point.y = point.y;
    geo_point.z = point.z;
    point_cloud.points.push_back(geo_point);
  }
  point_cloud.header.frame_id = "camere_link";
  point_cloud.header.stamp = ros::Time::now();
  sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
  point_cloud_pub.publish(point_cloud2);
}
void PubTf(const Eigen::Matrix4d& pose1) {
  Eigen::Matrix3d base_link2_camera =(
      Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0, 0, 1)) *
      Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1, 0, 0))).toRotationMatrix();
  geometry_msgs::TransformStamped tf_trans;

  Eigen::Matrix4d map_to_camera_world = Eigen::Matrix4d::Identity();
  map_to_camera_world.block(0,0,3,3) = base_link2_camera;
  Eigen::Matrix4d pose = map_to_camera_world*pose1;
  Eigen::Matrix3d ratation  = pose.block(0,0,3,3);
  Eigen::Quaterniond q(ratation);
  tf_trans.header.stamp = ros::Time::now();
  tf_trans.header.frame_id = "map";
  tf_trans.child_frame_id = "camere_link";
  tf_trans.transform.translation.x = pose(0,3);
  tf_trans.transform.translation.y = pose(1,3);
  tf_trans.transform.translation.z = pose(2,3);
  tf_trans.transform.rotation.x = q.x();
  tf_trans.transform.rotation.y = q.y();
  tf_trans.transform.rotation.z = q.z();
  tf_trans.transform.rotation.w = q.w();
  tf_broadcaster->sendTransform(tf_trans);
  

}
void PubFeatur(const cv::Mat&featureImag )
{
  sensor_msgs::Image img;
  std_msgs::Header header;
  header.frame_id =  "base_link" ;
  header.stamp  =  ros::Time::now();
//  cv_bridge::CvImage()
  sensor_msgs::ImagePtr msg= cv_bridge::CvImage(header,"bgr8",featureImag).toImageMsg();
  feat_img_pub.publish(msg);

}
void stereo_tracker(const std::pair<cv::Mat, cv::Mat>& stereo_imag) {
  Eigen::Matrix4d pose =
      StereoTrack_.Track(stereo_imag.first, stereo_imag.second);
  PubTf(pose);
  std::vector<cv::Point3f> points = StereoTrack_.GetTrackPoints();
  PubPointCloud2(points);
  PubFeatur(StereoTrack_.GetVisuImag());
}

int main(int argc,char** argv)
{

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc,argv,"stero_test");
   tf_broadcaster = new tf::TransformBroadcaster;
  std::string left_camera_topic;
  std::string right_camera_topic;
  ros::NodeHandle private_node("~");
  ros::NodeHandle nh;
  point_cloud_pub  = nh.advertise<sensor_msgs::PointCloud2>("track_point",2);
  feat_img_pub = nh.advertise<sensor_msgs::Image>("featr_img",2);
  private_node.param<std::string>("left_camera_topic",left_camera_topic,"/left_camera");
  private_node.param<std::string>("right_camera_topic",right_camera_topic,"/right_camera");
   

  ROS_INFO("\033[1;33m stero_test \033[0m \n");
  if(argc!=2){
    ROS_INFO("please input bag file");
    return EXIT_FAILURE;
  }



  std::string inputbag_name(argv[1]);

  std::thread thread_bag([&]() {
    ROS_INFO("start thread_bag"); 
    rosbag::Bag in_bag;
    in_bag.open(inputbag_name, rosbag::bagmode::Read);
    rosbag::View view(in_bag);
    rosbag::View::const_iterator view_iterator = view.begin();
    std::pair<cv::Mat,cv::Mat> stereo_imag;
    for (auto view_iterator = view.begin(); view_iterator != view.end();
         view_iterator++) {
      rosbag::MessageInstance msg = *view_iterator;

      if (msg.isType<sensor_msgs::CompressedImage>()) {
         
        if (msg.getTopic() == left_camera_topic) {
          sensor_msgs::CompressedImagePtr msg_ptr =
              msg.instantiate<sensor_msgs::CompressedImage>();
          cv::Mat matrix = cv::imdecode(cv::Mat(msg_ptr->data), 0);
          matrix.copyTo(stereo_imag.first);
        //  std::cout<<"left_camera_topic"<<std::endl;
          cv::Mat clolor_map =matrix.clone();
          cv::cvtColor(matrix,clolor_map,cv::COLOR_GRAY2RGB);

        }
        if (msg.getTopic() == right_camera_topic) {
      //   std::cout<<"right_camera_topic"<<std::endl;
          sensor_msgs::CompressedImagePtr msg_ptr =
              msg.instantiate<sensor_msgs::CompressedImage>();
          cv::Mat matrix = cv::imdecode(cv::Mat(msg_ptr->data),0);
          matrix.copyTo(stereo_imag.second);
       //  cv::imshow("right_camera", matrix);
       //   cv::waitKey(1);
        }
      }
      if(!stereo_imag.first.empty() && !stereo_imag.second.empty()){
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout<<"stereo_tracker"<<std::endl;
        stereo_tracker(stereo_imag);
        stereo_imag =  std::make_pair<cv::Mat,cv::Mat>(cv::Mat(),cv::Mat());
      }

    }
  });
  ros::Rate rate(10);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
  ros::shutdown();

}


