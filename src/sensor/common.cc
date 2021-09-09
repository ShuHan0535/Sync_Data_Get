#include "sensor/common.h"

//linux系统头文件
#include <errno.h>
#include <sys/stat.h>
#include <errno-base.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

namespace sensor{
    //构造函数 确定传感器名 传感器采集的目录 
    SensorAbstraction::SensorAbstraction(const char* sensor_name,const int& director_num){
        file_p=new file::FileOpt(sensor_name,director_num);
    };
    SensorAbstraction::~SensorAbstraction(){
        delete file_p;
    }
bool SensorAbstraction::RunOK =true;
}

namespace file{

  FileOpt::FileOpt(const char* sensor_name,const int& directory_num):
  sensor_name_(sensor_name),directory_num_(directory_num),file_fd_(-1),direct_name_("FILE"){};

  FileOpt::~FileOpt(){
      if(file_fd_ != -1){
          close(file_fd_);//关闭文件
      }
  }

  int FileOpt::Init(){
      //创建目录 目录名为FILE+目录号
      direct_name_.push_back(static_cast<char>(directory_num_+'0'));
      int ret=0;
      if((ret = mkdir(direct_name_.data(),0777))==-1){
          if( errno == EPERM ){
              perror("mkdir:");
              return -1;
          }
      }
      //创建文件
      std::string file_name=direct_name_+"/"+sensor_name_+"_file";
      if((file_fd_=open(file_name.data(),O_WRONLY|O_CREAT))==-1){
          perror("open:");  
          return -1;
      }
      return 0;
  }
  std::string& FileOpt::GetDirectName(){
      return direct_name_;
  }

}  

namespace memory
{
 
  MemOpt::MemOpt(const uint64_t phy_base_addr):phy_base_addr_(phy_base_addr),vir_base_addr_(0),mem_fd_(-1){

  };
  MemOpt::~MemOpt(){
    //已经打开了设备时 关闭设备
    if(mem_fd_!=1){
      close(mem_fd_);
    }
    //取消映射
    if(vir_base_addr_!=0){
      munmap(vir_base_addr_,PAGE_SIZE);
    }
  }

  int MemOpt::Init(){
    //设备已经被打开时 直接返回
    if(mem_fd_!=-1){
        return 0;
    }
    if((mem_fd_= open("/dev/mem", O_RDONLY | O_SYNC)) == -1)
	{
		perror("open /dev/mem:");
		return -1;
	}
	//建立映射
	uint64_t base = phy_base_addr_ & PAGE_MASK;//基地址
    if((vir_base_addr_ = (char *)mmap(NULL, PAGE_SIZE, PROT_READ, MAP_SHARED,mem_fd_, base))
    ==MAP_FAILED)
    {
        perror("mmap");
        return -1;
    }
    return 0;
  };
  char* MemOpt::GetMemoryPhy(const uint64_t& offset){
      return vir_base_addr_+offset;
  }


} // namespace memory