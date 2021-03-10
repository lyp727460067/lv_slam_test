#include <vector>
#include <tuple>
#include  <algorithm>
int main(int argc,char **argv)
{
int k = 2;
std::vector<std::pair<float,float>> points = {{3,3},{5,-1},{-2,4}};
std::vector<std::pair<float,std::pair<float,float>>> points_with_distance;

for (auto point : points) {
  float distance =
      sqrt(point.first * point.first + point.second * point.second);
  points_with_distance.push_back({distance, point});

}
std::sort(points_with_distance.begin(), points_with_distance.end(),
          [](std::pair<float, std::pair<float, float>> first,
             std::pair<float, std::pair<float, float>> sencode) {
            return first.first < sencode.first;
          });
  std::vector<std::pair<float,float>> results ;

  for (int i = 0; i < k; i++) {
    results.push_back(points_with_distance[i].second);
    std::cout<<points_with_distance[i].second.first<<" "<<points_with_distance[i].second.second<<std::endl;
  }
  
}
