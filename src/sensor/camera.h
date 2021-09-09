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
//本项目其他头文件
#include "include/camera_api.h"


namespace sensor{

class Camera :public SensorAbstraction
{
public:
    Camera(const char* sensor_name,const int& director_num,std::shared_ptr<memory::MemOpt> &data_addr_ptr,const uint64_t& camera_data_offset);
    ~Camera() override;
    int Init() override;
    int Run() override ;
private:

    int DataCollection();
    int DataProcessing();
    int DataStorage();

    struct camera_data;//camera数据帧结构体
    struct camera_data* data_process_;
    char* camera_addr_;//camera数据地址
    int camera_irq_fd_;//camera中断设备描述符
    std::shared_ptr<memory::MemOpt> data_addr_ptr_;//camera数据地址
    uint64_t camera_data_offset_;

    /*MDSDK配置参数*/
    int                     i_camera_counts_;
    tSdkCameraDevInfo       t_camera_enum_list_;
    int                     h_camera_;
    tSdkCameraCapbility     t_capability_;
    tSdkFrameHead           s_frame_info_;
    BYTE*			        pby_buffer_;
    tSdkImageResolution     s_image_size_;
    unsigned char           * g_prgb_buffer_;

};
}