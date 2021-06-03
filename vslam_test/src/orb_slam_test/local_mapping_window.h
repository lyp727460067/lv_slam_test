#ifndef _LOCAL_MAPPING_WINDOW_H
#define  _LOCAL_MAPPING_WINDOW_H
#include <vector>
#include "key_frame.h"
struct   LocalMappingWindowOption
{
	int windows_size;


};
struct LocalMappingResult
{

};

class LocalMappingWindow
{
 public:
 LocalMappingWindow(const LocalTrajectorBuilderOption & option);
LocalMappingResult Insert(std::shared_ptr<KeyFrame> frame_data);
 private:
 std::vector<std::shared_ptr<KeyFrame>> data_;
LocalTrajectorBuilderOption option_;


};

#endif