#pragma once

#include "sensor/common.h"
#include "sensor/user.h"
//Linux系统头文件
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h> 
#include <unistd.h>
//C头文件
#include <string.h> 
//C++头文件
#include <memory>

namespace sensor{

class Imu :public SensorAbstraction
{
public:
    Imu(const char* sensor_name,const int& director_num,std::shared_ptr<memory::MemOpt> &data_addr_ptr,const uint64_t& imu_data_offset);
    ~Imu() override;
    int Init() override;
    int DataCollection();
    int DataProcessing();
    int DataStorage();
    int Run() override ;
private:
    struct imu_data;//lidar数据帧结构体
    struct imu_data* data_process_;
    char* imu_addr_;//imu数据地址
    int imu_irq_fd_;//imu中断设备描述符
    std::shared_ptr<memory::MemOpt> data_addr_ptr_;//imu数据地址
    uint64_t imu_data_offset_;
};
}