#include "sensor/camera.h"
#include "sensor/lidar.h"
#include "sensor/imu.h"
#include "sensor/gnss.h"
int main(int argc,char** argv){
    //解析参数

    //创建管理内存类对象
    auto  memopt_p=std::make_shared<memory::MemOpt>(0x20000000);
    //创建各个传感器类对象
    auto lidar_p = std::make_shared<sensor::Lidar>("eth1",2368,"lidar");
    auto imu_p = std::make_shared<sensor::Imu>("imu",1,memopt_p,0);
    auto gnss_p = std::make_shared<sensor::Gnss>("gnss",1,memopt_p,0x70);
    auto camera_p = std::make_shared<sensor::Camera>("camera",1,memopt_p,0x180);
    //开始执行
    lidar_p->StartThread();
    imu_p->StartThread();
    gnss_p->StartThread();
    camera_p->StartThread();
    
}



