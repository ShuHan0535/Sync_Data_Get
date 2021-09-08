#include "sensor/gnss.h"


namespace sensor{

    Gnss::Gnss(const char* sensor_name,const int& director_num,std::shared_ptr<memory::MemOpt> &data_addr_ptr,const uint64_t& gnss_data_offset):
    SensorAbstraction(sensor_name,director_num),data_addr_ptr_(data_addr_ptr),gnss_data_offset_(gnss_data_offset),gnss_irq_fd_(-1){
        data_process_=new gnss_data();
        data_raw_ = new char[BUFFSIZE];
        gga_save_line_ = new char[BUFFSIZE];
        gga_update_flag_=false;
        vtg_save_line_=new char[BUFFSIZE];
        vtg_update_flag_=false;
        heading_save_line_ = new char[BUFFSIZE];
        heading_update_flag_=false;
        gps_gga_quality_ = 0;
        gps_gga_num_sats_= 0;
        gps_update_flag_ = false;
        gps_heading_ = 0.0 ;
        gnss_data_active_flag_ = false;
        gps_gga_latitude_ = 0;
        gps_gga_longitude_ = 0;
        gps_gga_hdop_ = 0;
        gps_gga_altitude_ = 0 ;
        gps_gga_wgs_alt_ = 0;
        gps_vtg_speed_ = 0;
        gps_vtg_course_ = 0;
        gps_heading_ = 0;
        gps_pitch_ = 0;
    };

    Gnss::~Gnss(){
        delete data_process_;
        delete [] data_raw_;
        delete [] gga_save_line_;
        delete [] vtg_save_line_;
        delete [] heading_save_line_;
        if(gnss_irq_fd_ != -1){
            close(gnss_irq_fd_);
        }
    };

    int Gnss::Init(){
        //配置gnss中断
        if((gnss_irq_fd_=open("/dev/GNSS_irq",O_RDWR)) == -1){
            perror("gnss open:");
            return -1;
        }
        //获取gnss储存数据的DDR基地址
        gnss_addr_=data_addr_ptr_->GetMemoryPhy(gnss_data_offset_);

        //初始化本地储存的文件 初始化失败返回-1 
		if(file_p->Init()){
			return -1;
		}
        return 0;
    };

    int Gnss::DataCollection(){
        //等待gnss数据到来
        ioctl(gnss_irq_fd_, GNSS_IRQ,1);
        memcpy(data_process_,gnss_addr_,BUFFSIZE);
        return 0;
    }

    int Gnss::DataProcessing(){
        uint32_t gnss_us_stamp=((static_cast<uint8_t>(data_raw_[255])<<24)+(static_cast<uint8_t>(data_raw_[254])<<16)+
        (static_cast<uint8_t>(data_raw_[253])<<8)+(static_cast<uint8_t>(data_raw_[252])))/100;
        uint32_t gnss_s_stamp=(static_cast<uint8_t>(data_raw_[251])<<8)+(static_cast<uint8_t>(data_raw_[250]));
        uint8_t gnss_length=static_cast<uint8_t>(data_raw_[249]);//获取报文长度
        memset(&data_raw_[gnss_length],'\0',BUFFSIZE-gnss_length);
        //$GPGGA
        if(strncmp(data_raw_,"$GPGGA",6) == 0&&gga_update_flag_ == false)
		{ 
			//根据','的个数来判断报文正确性 总共有14个','
			//报文信息以'!'结尾
			if(CharCounter(data_raw_,',') == 14)
			{
				//保存GGA报文数据到gga_save_line
				strncpy(gga_save_line_,data_raw_,gnss_length);
				//GNSS_buffer
				data_process_->gga_s_stamp=gnss_s_stamp;
				data_process_->gga_us_stamp=gnss_us_stamp;
				memset(data_raw_,'\0',BUFFSIZE);
				gga_update_flag_=true;
			}		
		}

        if(strncmp(data_raw_,"$GPVTG",6) == 0)//$GPVTG
		{	
			if(CharCounter(data_raw_,',') == 9 && (vtg_update_flag_ == 0)) 
			{
				strncpy(vtg_save_line_,data_raw_,gnss_length);
				data_process_->vtg_s_stamp=gnss_s_stamp;
				data_process_->vtg_us_stamp=gnss_us_stamp;
				memset(data_raw_,'\0',BUFFSIZE);
				vtg_update_flag_ = true ;
			}
		}

        if(strncmp(data_raw_,"#HEADING",8) == 0)//$HEADING
		{
			//printf("enter heading\n");
			if(CharCounter(data_raw_,',') == 25&& (heading_update_flag_ == 0)) 
			{					
				strncpy(heading_save_line_,data_raw_,gnss_length);
				data_process_->heading_s_stamp=gnss_s_stamp;
				data_process_->heading_us_stamp=gnss_us_stamp;
				memset(data_raw_,'\0',BUFFSIZE);
				heading_update_flag_ = true ;
			}
		}
        //当三个报文数据都更新时
		if(gga_update_flag_&&vtg_update_flag_&&heading_update_flag_)
		{	//解析报文数据
			GnssUpdate();								  		
		}    
    }

    int Gnss::DataStorage(){
        if(gps_update_flag_ == true)
		{
            int ret;
			//储存数据
			if(( ret = write(file_p->GetFd(),data_process_,sizeof(gnss_data)) )<0)
            {
                perror("Gnss write:");
                return -1;
            }
            gps_update_flag_ = false;
		}
        return 0;
    }

    int Gnss::Run(){
        while(RunOK){
			if(DataCollection()||DataProcessing()||DataStorage()){
				return -1;
			}
		}
		return 0;
    }

    uint32_t Gnss::CharCounter(const char* s,const char& c){
        unsigned int sum=0;
	    while(*s!='!')
	    {
		    if(*s++==c)
			sum++;
	    }
	    return sum;
    }

    void Gnss::GnssUpdate(){
        bool gps_sample = false;
        if(gga_update_flag_)
        {
     	    //更新位置信息
            GpggaUpdate();
            gga_update_flag_=false;   
            gps_sample = true;
        }
    
        if(vtg_update_flag_)
        {
            GpvtgUpdate();
            vtg_update_flag_ = false;
            gps_sample = true;
        } 

        if(heading_update_flag_)
        {
            HeadingUpdate();
            heading_update_flag_ = false;
            gps_sample = true;
        } 
     
        if(gps_sample==true)
        {
            gps_sample = false;
		    //gps_gga_quality表示GPS信号质量        gps_gga_num_sats为可用的卫星数量
            if((gps_gga_quality_==4) && (gps_gga_num_sats_>=4))//RTK锁定状态
            //if((gps_gga_quality==1) && (gps_gga_num_sats>=4))//单点模式
            { 	
                gps_update_flag_ = true ;
			    //用第一次GPS的数据（经纬度和高程）下的地球中心坐标系到NED坐标系的旋转矩阵
			    gnss_data_active_flag_ = true;//GNSS数据有效标志位
                GnssDataInit();
                Gnss2xyz(data_process_->current_xyz);    //update current_xyz指NED坐标系中的位置坐标
                Gnss2uvw(data_process_->current_uvw);    //update speed指NED坐标系中的对地速度
                data_process_->heading = gps_heading_;
			    data_process_->gps_sats = gps_gga_num_sats_;
			    data_process_->gps_mod = gps_gga_quality_;
            }
        }
    }
    void Gnss::GpggaUpdate(void)
    {
        int lat = 0 ;
        double lat_min = 0.0;
        char * line ;
		//用','分割报文数据 line指向第一个','
        line = index(gga_save_line_,',');
        line++; 
		//第一个报文数据为UTC时间
		/*strtod函数将字符串转换为数字
		//返回  双精度数字 			将要转换的字符串首指针 		    指向符合条件的最后一个字符数字的指针
		double         strtod (constchar* str,         char** endptr)*/
		//gga时间精确到ms，作为位置点采集时间使用 
        double gps_gga_time =  strtod(line,&line);
        int hour = static_cast<int>((gps_gga_time/10000.0));
        int min = static_cast<int>(((gps_gga_time-hour*10000)/100.0));
        float sec   = gps_gga_time-hour*10000-min*100;
        gps_gga_time =  (min * 60 + sec)*100;
        data_process_->gnss_time_stamp=(min*60+sec)*1000;
        
        line++;//line++后又指向了下一个可以转化为数字的字符地址、
        //维度
        gps_gga_latitude_= strtod( line, &line);
        lat = (int)(gps_gga_latitude_*0.01);
        lat_min = gps_gga_latitude_ - lat*100.0;
        gps_gga_latitude_ = lat + lat_min / 60.0;
        
        
		line++;
		if( *line == 'S' )      // Find the sign for the lattitude.  South is negative
        {
            //标记为南纬为负，北纬为正
	 	    gps_gga_latitude_ *= -1;
        }
		line = index(line,',');               
        //经度
        line++;
        gps_gga_longitude_ = strtod( line, &line);
		lat= (int)(gps_gga_longitude_*0.01);
		lat_min= gps_gga_longitude_ - lat*100 ;
		gps_gga_longitude_ = lat + lat_min / 60.0;
		
        line++;
        //标记为西经为负 东经为正
		if( *line == 'W' )
        {
            gps_gga_longitude_ *= -1;
        }
		
		line = index(line,',');// Skip ,W,
	
        line++;
	    gps_gga_quality_ = strtol( line, &line, 10 );
	    line++;		// Skip ,
	    gps_gga_num_sats_ = strtol( line, &line, 10 );
	    line++;		// Skip ,
	    gps_gga_hdop_ = strtod( line, &line );
	    line++;		// Skip ,
	    gps_gga_altitude_ = strtod( line, &line );  
	    line += 3;	// Skip ,M,
	    gps_gga_wgs_alt_ = strtod( line, &line );  
	    line += 3;	// Skip ,M,
    }

    void Gnss::GpvtgUpdate(){
        char * line=NULL ; 
        line = index(vtg_save_line_,',');
        if(!line){
            return ;
        }
        line++;//skip ,
        //Track made good, degrees True
        gps_vtg_course_ = strtod( line, &line );
        line = index(line,'N');
        if(!line){
            return;
        }
        line += 2;
        // Speed, kilometres/hour
        gps_vtg_speed_ = strtod( line, &line ) * 1000.0 / 3600.0;
    }

    void Gnss::HeadingUpdate(){
        char * line ;
        line = index(heading_save_line_,',');
        line++; 
        line = index(line,',');
        line++; 
        line = index(line,',');
        line++; 
        line = index(line,',');
        line++; 
        line = index(line,',');
        line++; 
        line = index(line,',');
        line++; 
        line = index(line,',');
        line++; 
        line = index(line,',');
        line++; 
        line = index(line,',');
        line++; 
        line = index(line,',');
        line++; 
        line = index(line,',');
        line++; 
        line = index(line,',');
        line++; 
	    //航向角
	    gps_heading_ = strtod(line,&line);
	    if(gps_heading_>180.0f){
            gps_heading_ = gps_heading_-360.0f;
        }
	    line++; 
	    //俯仰角pitch
	    gps_pitch_ = strtod(line,&line);
    }   

    void Gnss::GnssDataInit(void)
    {
   	    static bool first_entry = true ;
        if(first_entry)
        {
            // wws 20110530: we have set the zero_position012//   
            //单位转换 deg->rad 仍然是经纬度和航向
            double zero_position0 = gps_gga_latitude_*C_DEG2RAD;//经度
            double zero_position1 = gps_gga_longitude_*C_DEG2RAD;//纬度
            double zero_position2 = gps_gga_altitude_+gps_gga_wgs_alt_;//大地水平高程信息
            //PEG
            zero_position_[0]=zero_position0;
            zero_position_[1]=zero_position1;
            zero_position_[2]=zero_position2;
            f_ = ((C_WGS84_a - C_WGS84_b) / C_WGS84_a);
            e_ = sqrt( 2*f_ - f_*f_ );
            double N = C_WGS84_a/sqrt( 1 - e_*e_*pow(sin(zero_position0),2));
		    
            double	clat = cos(zero_position0);
            double	clon = cos(zero_position1);
            double	slat = sin(zero_position0);
            double	slon = sin(zero_position1);
		    //ECEF转NED 旋转矩阵 RE
		    Re2t_[0][0] = -slat*clon;
		    Re2t_[0][1] = -slat*slon;
		    Re2t_[0][2] =  clat;

		    Re2t_[1][0] = -slon;
		    Re2t_[1][1] =  clon;
		    Re2t_[1][2] =  0.0;

		    Re2t_[2][0] = -clat*clon;
		    Re2t_[2][1] = -clat*slon;
		    Re2t_[2][2] = -slat;
        }
        first_entry=false;
    }

    void Gnss::Gnss2xyz(double *llh)
    {

        llh[0] = gps_gga_latitude_*C_DEG2RAD;
        llh[1] = gps_gga_longitude_*C_DEG2RAD;
        llh[2] = gps_gga_altitude_+gps_gga_wgs_alt_;
        //经纬度转地球中心ECEF坐标系
        Llh2ECEF( llh );
        
        llh[0] = llh[0] - zero_position_ECEF_[0];
        llh[1] = llh[1] - zero_position_ECEF_[1];
        llh[2] = llh[2] - zero_position_ECEF_[2];
	    //ECEF转NED
        //PG=RE*(PEG-PEG0)	
        ECEF2Tangent(llh);
        //printf("x y z of NED axis is %f,  %f ,%f\r\n  ",llh[0],llh[1],llh[2]);
    }




    void Gnss::Llh2ECEF( double *llh){
        double value[3];
	    double	N = C_WGS84_a/sqrt( 1 - e_*e_*pow(sin(llh[0]),2));

	    value[0]= (N + llh[2]) * cos(llh[0]) * cos(llh[1]);
        value[1]= (N + llh[2]) * cos(llh[0]) * sin(llh[1]);
	    value[2]= (N*(1-e_*e_) + llh[2]) * sin(llh[0]);
        llh[0] = value[0];
        llh[1] = value[1];
        llh[2] = value[2];
    }

    void Gnss::ECEF2Tangent(double *ECFF){
        double value[3];
        value[0] = Re2t_[0][0]*ECFF[0]+Re2t_[0][1]*ECFF[1]+Re2t_[0][2]*ECFF[2];
        value[1] = Re2t_[1][0]*ECFF[0]+Re2t_[1][1]*ECFF[1]+Re2t_[1][2]*ECFF[2];
        value[2] = Re2t_[2][0]*ECFF[0]+Re2t_[2][1]*ECFF[1]+Re2t_[2][2]*ECFF[2];
        
        ECFF[0] = value[0];
        ECFF[1] = value[1];
        ECFF[2] = value[2];
    }
    void Gnss::Gnss2uvw(double *vel_NED){
        int i;
        static double last_Z = 0.0;               
        double vel_z_current =0.0 ;
        static int last_gps_gga_num_sats =0 ;
        //GPS
        vel_NED[0] = gps_vtg_speed_ * cos( gps_vtg_course_ * C_DEG2RAD );
        vel_NED[1]  = gps_vtg_speed_ * sin( gps_vtg_course_ * C_DEG2RAD );

        //vel_NED[0] = gps_vtg_speed * cos( cps_theta[2] * C_DEG2RAD );
        //vel_NED[1]  = gps_vtg_speed * sin( cps_theta[2] * C_DEG2RAD );
    
        if(last_gps_gga_num_sats != gps_gga_num_sats_)
        {
            last_gps_gga_num_sats = gps_gga_num_sats_;
            last_Z = data_process_->current_xyz[2];
            return ;
        }   
        vel_z_current = (data_process_->current_xyz[2] - last_Z)*GPS_FREQ ;
    
        float sum_vel_z =0.0; 
        static float window_vel_z[FILTER_LEN] = {0.0,0.0} ; 
    
        for( i =FILTER_LEN-1 ; i != 0 ; --i )
        {
            window_vel_z[i] = window_vel_z[i-1];
            sum_vel_z += window_vel_z[i];
        }
    
        window_vel_z[0] = vel_z_current ;
        sum_vel_z += window_vel_z[0] ;
        vel_NED[2] = sum_vel_z/FILTER_LEN ;        
 
        last_Z = data_process_->current_xyz[2];
     
    }
    struct Gnss::gnss_data{
        double current_xyz[3];   
	    double current_uvw[3];
	    double heading ;//航向
	    int gps_sats;//GPS搜星数
	    int gps_mod;//GPS状态
	    uint32_t gga_s_stamp;
	    uint32_t gga_us_stamp;//us时间戳

	    uint32_t vtg_s_stamp;
	    uint32_t vtg_us_stamp;//us时间戳
	
	    uint32_t heading_s_stamp;
	    uint32_t heading_us_stamp;//us时间戳
	
	    uint32_t gnss_time_stamp;//ns时间戳
    };
};