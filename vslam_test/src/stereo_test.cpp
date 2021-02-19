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
void stereo_tracker(const std::pair<cv::Mat,cv::Mat>& stereo_imag)
{


}


int main(int argc,char** argv)
{
  ros::init(argc,argv,"stero_test");
  tf::TransformBroadcaster* tf_broadcaster = new tf::TransformBroadcaster;
  std::string left_camera_topic;
  std::string right_camera_topic;
  ros::NodeHandle private_node("~");
  private_node.param<std::string>("left_camera_topic",left_camera_topic,"/left_camera");
  private_node.param<std::string>("right_camera_topic",right_camera_topic,"/right_camera");
   

  ROS_INFO("\033[1;33m stero_test \033[0m \n");
  if(argc!=2){
    ROS_INFO("please input bag file");
    return EXIT_FAILURE;
  }


  ros::NodeHandle nh;
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
              msg.instantiate<sensor_msgs::CompressedImage>();\
          cv::Mat matrix = cv::imdecode(cv::Mat(msg_ptr->data), 1);
          matrix.copyTo(stereo_imag.first);
         // cv::imshow("left_camera", matrix);
         // cv::waitKey(1);
        }
        if (msg.getTopic() == right_camera_topic) {
          sensor_msgs::CompressedImagePtr msg_ptr =
              msg.instantiate<sensor_msgs::CompressedImage>();
          cv::Mat matrix = cv::imdecode(cv::Mat(msg_ptr->data), 1);
          matrix.copyTo(stereo_imag.second);
         // cv::imshow("right_camera", matrix);
         // cv::waitKey(1);
        }
      }
      if(!stereo_imag.first.empty()&& !stereo_imag.second.empty()){
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