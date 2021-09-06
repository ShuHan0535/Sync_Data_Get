#include "sensor/imu.h"

namespace sensor{
    Imu::Imu(const char* sensor_name,const int& director_num,std::shared_ptr<memory::MemOpt> &data_addr_ptr,const uint64_t& imu_data_offset):
    SensorAbstraction(sensor_name,director_num),data_addr_ptr_(data_addr_ptr),imu_data_offset_(imu_data_offset),imu_irq_fd_(-1){
        data_process_=new imu_data();
    };
    Imu::~Imu(){
        delete data_process_;
        if(imu_irq_fd_ != -1){
            close(imu_irq_fd_);
        }
    }

    int Imu::Init(){
        //配置imu中断
        if((imu_irq_fd_=open("/dev/IMU_irq",O_RDWR)) == -1){
            perror("imu open:");
            return -1;
        }
        //获取imu储存数据的DDR基地址
        imu_addr_=data_addr_ptr_->GetMemoryPhy(imu_data_offset_);

        //初始化本地储存的文件 初始化失败返回-1 
		if(file_p->Init()){
			return -1;
		}
    }

    int Imu::DataCollection(){
        //等待imu数据到来
        ioctl(imu_irq_fd_, IMU_IRQ,1);
        memcpy(data_process_,imu_addr_,104);
        ++data_process_->imu_number;
        return 0;
    }
    int Imu::DataProcessing(){
        //imu数据处理
        return 0;
    }
    int Imu::DataStorage(){
        int ret=0;
        if( (ret = write(file_p->GetFd(),data_process_,sizeof(imu_data)) < 0) ){
            return -1;
        }
        return 0;    
    }

    struct Imu::imu_data{
        imu_data():imu_number(0){};
        int16_t imu_raw_data[48];//imu数据
        uint32_t imu_s_time;//秒时间戳
        uint32_t imu_ns_time;//ns时间戳
	    float imu_number;//imu数据序列号
    };

}