#pragma once

#include "sensor/common.h"
//linux系统文件
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <unistd.h>
//C头文件
#include <string.h>
//C++头文件
#include <string>
#include <iostream>

namespace sensor{

class Lidar :public SensorAbstraction
{
public:
    Lidar(const char* nic,const uint16_t& port,const char* sensor_name,const int& director_num);
    ~Lidar() override;
    int Init() override;
    int Run() override ;
private:

    int DataCollection();
    int DataProcessing();
    int DataStorage();

    struct lidar_data;//lidar数据帧结构体
    struct lidar_data* data_process_;
    static const size_t BUFFSIZE = 1248;//一帧Lidar数据的大小
    struct sockaddr_in server_addr_;   //服务器网络地址结构体
    struct sockaddr_in remote_addr_;   //客户端网络地址结构体
    uint8_t* data_raw_;//数据接收缓存区
    int server_sockfd_;//网络设备描述符
    socklen_t sin_size_;
    char* nic_name_;//网卡名
    uint16_t port_num_;//lidar接入本地的端口号
};



    
   
    
}