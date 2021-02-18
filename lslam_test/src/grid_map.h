#ifndef GRID_MAP_H
#define GRID_MAP_H
#include <nav_msgs/OccupancyGrid.h>

#include <ostream>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "opencv2/core.hpp"
class GridMap {
 public:
  class Options {};

 private:
};
namespace std {
ostream& operator<<(ostream& in, Eigen::Vector3i& data) {
  in << data.x() << " " << data.y() << " " << data.z() << std::endl;
  return in;
}
}  // namespace std
class OccupancyGridMap : public GridMap {
 public:
  OccupancyGridMap() {
    cv_map_data = cv::Mat(1000, 1000, CV_32FC1, cv::Scalar(-1));
    Occu_map.header.frame_id = "map";
    Occu_map.info.height = 1000;
    Occu_map.info.width = 1000;
    Occu_map.info.origin.orientation.x = 0;
    Occu_map.info.origin.orientation.y = 0;
    Occu_map.info.origin.orientation.z = 0;
    Occu_map.info.origin.orientation.w = 1;
    Occu_map.info.resolution = 0.05;
    Occu_map.info.origin.position.x =
        -0.5 * Occu_map.info.width * Occu_map.info.resolution;
    Occu_map.info.origin.position.y =
        -0.5 * Occu_map.info.height * Occu_map.info.resolution;
    Occu_map.info.origin.position.z = 0;
    Occu_map.data.resize(1000 * 1000);
  }
  OccupancyGridMap(const Options options) {}

  void Insert(std::vector<Eigen::Vector3f> range_points, Eigen::Matrix4f pose) {
    Eigen::Vector3i Origin = ToGrid(pose.block<3, 1>(0, 3));
    for (auto point : range_points) {
      Eigen::Vector3i GridPoint = ToGrid(point);
      UpdateReturnMap(GridPoint);
      UpdateMissMap(GetTwoPointIndex(Origin, GridPoint));
    }
  }
  inline float ProbabilityFromOdds(const float odds) {
    return odds / (odds + 1.f);
  }
  nav_msgs::OccupancyGrid GetOccuMap() {
    for (int i = 0; i < cv_map_data.rows; i++) {
      for (int j = 0; j < cv_map_data.cols; j++) {
        Occu_map.data[i * Occu_map.info.width + j] =
            ProbabilityFromOdds(cv_map_data.at<float>(j, i)) * 100;
        if (Occu_map.data[i * Occu_map.info.width + j] >= 100)
          Occu_map.data[i * Occu_map.info.width + j] = 100;
        if (Occu_map.data[i * Occu_map.info.width + j] <= 0)
          Occu_map.data[i * Occu_map.info.width + j] = -1;
      }
    }
    std::cout << "OccupanyGrid" << std::endl;
    Occu_map.header.stamp = ros::Time::now();
    return Occu_map;
  }

 private:
  Eigen::Vector3i ToGrid(Eigen::Vector3f point) {
    return Eigen::Vector3i(point.x() / 0.05 + 500, point.y() / 0.05 + 500,
                           point.z() / 0.05);
  }
  void UpdateReturnMap(Eigen::Vector3i point) {
    if (cv_map_data.at<float>(point.x(), point.y()) == -1) {
      cv_map_data.at<float>(point.x(), point.y()) =
          hit_problity / (1 - hit_problity);
    } else {
      cv_map_data.at<float>(point.x(), point.y()) *=
          hit_problity / (1 - hit_problity);
    }

    if (cv_map_data.at<float>(point.x(), point.y()) >= 10000) {
      cv_map_data.at<float>(point.x(), point.y()) = 10000;
    }
  }

  void UpdateMissMap(std::vector<Eigen::Vector3i> miss) {
    for (auto point : miss) {
      if (cv_map_data.at<float>(point.x(), point.y()) == -1) {
        cv_map_data.at<float>(point.x(), point.y()) =
            miss_problity / (1 - miss_problity);
      } else {
        cv_map_data.at<float>(point.x(), point.y()) *=
            miss_problity / (1 - miss_problity);
      }
      if (cv_map_data.at<float>(point.x(), point.y()) <= 0.1) {
        cv_map_data.at<float>(point.x(), point.y()) = 0.1;
      }
    }
  }
  std::vector<Eigen::Vector3i> GetTwoPointIndex(Eigen::Vector3i first_point,
                                                Eigen::Vector3i second_point) {
    int x0 = first_point.x();
    int y0 = first_point.y();
    int x1 = second_point.x();
    int y1 = second_point.y();
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2, e2;
    std::vector<Eigen::Vector3i> line_point;
    bool first = true;
    for (;;) {
      if (x0 == x1 && y0 == y1) break;
      if (!first) {
        Eigen::Vector3i index(x0, y0, 0);
        line_point.push_back(index);
      } else {
        first = false;
      }

      e2 = err;
      if (e2 > -dx) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dy) {
        err += dx;
        y0 += sy;
      }
    }
    return line_point;
  }
  int map_with = 1000;
  nav_msgs::OccupancyGrid Occu_map;
  Eigen::AlignedBox2i known_cells_box_;
  void UpdataBound() {}
  cv::Mat cv_map_data;  //(10000,10000,CV_32FC1,cv::Scalar(-1));
  std::vector<float> map_data_;
  const double hit_problity = 0.6;
  const double miss_problity = 0.4;
};

#endif
