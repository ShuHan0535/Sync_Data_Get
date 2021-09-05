#pragma once
#include <string>
namespace sensor{
//传感器抽象层
class SensorAbstraction
{
  public:
    //构造函数 确定传感器名 传感器采集的目录 
    SensorAbstraction(const char* sensor_name,const int& director_num);
    virtual int Init()=0;//初始化 
    virtual int Run()=0;
    virtual ~SensorAbstraction();
    static bool RunOK;
    file::FileOpt* file_p;//采集该传感器数据对应的文件名
};
}
namespace file{
  //文件操作类 用于储存传感器的数据的文件的创建 数据的保存
class FileOpt{
  public:
  FileOpt(const char* sensor_name,const int& director_num);
  ~FileOpt();
  int& GetFd();
  int Init();
  private:
    std::string sensor_name_;//传感器名
    int directory_num_;//目录编号
    int file_fd_;//文件操作符

};
}