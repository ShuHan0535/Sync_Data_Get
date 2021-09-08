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
#include <math.h>
//C++头文件
#include <memory>

namespace sensor{
class Gnss : public SensorAbstraction{
public:
    Gnss(const char* sensor_name,const int& director_num,std::shared_ptr<memory::MemOpt> &data_addr_ptr,const uint64_t& imu_data_offset);
    ~Gnss() override;
    int Init() override;
    int Run() override ;
private:
    int DataCollection();
    int DataProcessing();
    int DataStorage();
    uint32_t CharCounter(const char* s,const char& c);
    void GnssUpdate();
    void GpggaUpdate();
    void GpvtgUpdate();
    void HeadingUpdate();
    void GnssDataInit();
    void Gnss2xyz(double *llh);
    void Llh2ECEF( double *llh);
    void ECEF2Tangent(double *ECFF);
    void Gnss2uvw(double *vel_NED);

    static const size_t BUFFSIZE = 260 ;
    static const double C_WGS84_a = 6378137.0;/* WGS-84 semimajor axis (m) */
    static const double C_WGS84_b = 6356752.3142;	  /* WGS-84 semiminor axis (m) */
    static const double C_DEG2RAD = 0.017453292519943295;
    static const int GPS_FREQ = 20;
    static const int FILTER_LEN = 3;

    struct gnss_data;//gnss数据帧结构体
    struct gnss_data* data_process_;
    char* gnss_addr_;//gnss数据地址
    int gnss_irq_fd_;//gnss中断设备描述符
    std::shared_ptr<memory::MemOpt> data_addr_ptr_;//gnss数据地址
    uint64_t gnss_data_offset_;    
    char* data_raw_;
    bool gga_update_flag_;
    char* gga_save_line_;
    bool vtg_update_flag_;
    char* vtg_save_line_;
    bool heading_update_flag_;
    char* heading_save_line_;
    int gps_gga_quality_;//星数
    int gps_gga_num_sats_;
    double gps_gga_hdop_;
    bool gps_update_flag_;
    double gps_heading_ ;
    volatile bool gnss_data_active_flag_;
    double gps_gga_latitude_;
    double gps_gga_longitude_;
    double gps_gga_altitude_ ;
    double gps_gga_wgs_alt_;
    double gps_vtg_speed_;
    double gps_vtg_course_;
    double gps_heading_;
    double gps_pitch_;
    double zero_position_[3];
    double f_;
    double e_;
    double zero_position_ECEF_[3];
    double Re2t_[3][3];
    };
}