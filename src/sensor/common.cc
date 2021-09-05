#include "sensor/common.h"

//linux系统头文件
#include <errno.h>
#include <sys/stat.h>
#include <errno-base.h>
#include <unistd.h>
#include <fcntl.h>
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
  sensor_name_(sensor_name),directory_num_(directory_num),file_fd_(-1){};

  FileOpt::~FileOpt(){
      if(file_fd_ != -1){
          close(file_fd_);//关闭文件
      }
  }

  int FileOpt::Init(){
      //创建目录 目录名为FILE+目录号
      std::string dir_name("FILE");
      dir_name.push_back(static_cast<char>(directory_num_+'0'));
      int ret=0;
      if((ret = mkdir(dir_name.data(),0777))==-1){
          if( errno == EPERM ){
              perror("mkdir:");
              return -1;
          }
      }
      //创建文件
      std::string file_name=dir_name+"/"+sensor_name_+"_file";
      if((file_fd_=open(file_name.data(),O_WRONLY|O_CREAT))==-1){
          perror("open:");  
          return -1;
      }
      return 0;
  }    

}    