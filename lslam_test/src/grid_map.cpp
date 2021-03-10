



int FindSencode(const std::vector<int>& datas)
{
  int big0 =  data[0],big1 = -10000000;
  for(auto data:datas){
    if(data> big0){
      big1 = big0;
      big0 = data; 
    }else {
      if(data>big1 ){
        big1 = data;
      }
    }
  }
  return big1;
}





using  fun  =  std::function<void(void)>;
表示返回空，参数列表空的函数

或者  lamda 表示方法
auto fun  =  [&](){};
（返回植默认是bool）
