#pragma once

#include <string>

#include "sensor/user.h"

//Linux系统头文件
#include <pthread.h>

namespace sensor{
//传感器抽象层
class SensorAbstraction
{
public:
    //构造函数 确定传感器名 传感器采集的目录 
    SensorAbstraction(const char* sensor_name,const int& director_num);
    virtual int Init()=0;
    virtual int Run()=0;
    virtual ~SensorAbstraction();
    static bool RunOK;
    file::FileOpt* file_p;//采集该传感器数据对应的文件类
    std::string sensor_name_;
    void ThreadInit(int sche_algo,int pth_priority);//初始化线程函数
    void StartThread();//启动传感器对应的采集线程

private:
    pthread_attr_t attr_;//线程属性结构体
    sched_param param_;//线程参数结构体
    pthread_t tid;//线程id    
};
//线程回调函数
void* callback(void* arg);


}

namespace file{
  //文件操作类 用于储存传感器的数据的文件的创建 数据的保存
class FileOpt{
public:
  FileOpt(const char* sensor_name,const int& director_num);
  ~FileOpt();
  int& GetFd();
  int Init();
  std::string& GetDirectName() ;
private:
    std::string sensor_name_;//传感器名
    int directory_num_;//目录编号
    int file_fd_;//文件操作符
    std::string direct_name_;
};
}

namespace memory
{
  //内存操作类
class MemOpt{
public:
  MemOpt(const uint64_t phy_bass_addr);
  ~MemOpt();
  int Init();
  uint64_t phy_base_addr_;//要映射的物理基地址
  int mem_fd_;//内存设备描述符
  char* vir_base_addr_;//得到的映射的虚拟基地址
  char* GetMemoryPhy(const uint64_t& offset);
};
} // namespace memory

