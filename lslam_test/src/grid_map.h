#ifndef GRID_MAP_H
#define GRID_MAP_H


class GridMap
{
  public:


  private:
 

};

class OccupancyGridMap:public GridMap
{
  public:
  OccupancyGridMap(float gird_size):grid_size_(gird_size){}

  private:
  float grid_size_ ;
  

};



#endif