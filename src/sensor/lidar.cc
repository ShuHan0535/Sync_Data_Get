#include "sensor/lidar.h"
namespace sensor{
    //构造函数
    Lidar::Lidar(const char* nic,const uint16_t& port_num,const char* sensor_name,const int& director_num):SensorAbstraction(sensor_name,director_num){
        //获取lidar绑定的网卡名
        nic_name_=new char[strlen(nic)+1];
        strcpy(nic_name_,nic);
        //获取lidar绑定的端口号
        port_num_=port_num;
        //申请数据接受缓存区
        data_raw_=new uint8_t[BUFFSIZE];
        memset(data_raw_,0,BUFFSIZE);
        //申请lidar数据帧
        data_process_=new lidar_data();
    }
    Lidar::~Lidar(){
        delete [] nic_name_;
        delete [] data_raw_;
        delete data_process_;
    }
    int Lidar::Init(){
        
	    int recv_size = 16 * 1024 * 1024;    //设置UDP缓存区大小为16M
	    memset(&server_addr_,0,sizeof(server_addr_)); //数据初始化--清零
	    server_addr_.sin_family=AF_INET; //设置为IP通信 ipv4
	    server_addr_.sin_addr.s_addr=INADDR_ANY;//服务器IP地址--允许连接到所有本地地址上
	    server_addr_.sin_port=htons(port_num_); //服务器端口号

	    //1.创建服务器端套接字--IPv4协议，面向无连接通信，UDP协议  SOCK_DGRAM为数据包套接字 对应UDP*/
	    if((server_sockfd_=socket(PF_INET,SOCK_DGRAM,0))<0)
	    {  
		    perror("socket error");
		    return -1;
	    }
	    //2.将套接字绑定到服务器的网络地址上
	    /*
	    int bind(int sockfd, const struct sockaddr *addr,socklen_t addrlen);	 
	    //	sockfd：套接字描述符   
	    //	addr:要绑定的地址信息	 
	    //	addrlen：地址信息的长度
	    //	返回值：int型，成功返回0，失败返回-1
	    */
	    if (bind(server_sockfd_,(struct sockaddr *)&server_addr_,sizeof(struct sockaddr))<0)
	    {
		    perror("bind error");
		    return -1;
	    }
	    //3.绑定LIDAR数据从nic_name网卡读取
	    struct ifreq interface;
        strncpy(interface.ifr_ifrn.ifrn_name, nic_name_, sizeof(nic_name_));
        if (setsockopt(server_sockfd_, SOL_SOCKET, SO_BINDTODEVICE, (char *)&interface, sizeof(interface))  < 0) {
           perror("SO_BINDTODEVICE failed");
           return -1;
        }
	    sin_size_=sizeof(struct sockaddr_in);
	    setsockopt(server_sockfd_,SOL_SOCKET, SO_RCVBUF, (const char *)&recv_size,sizeof(recv_size));
	    usleep(10000);
		//初始化本地储存的文件 初始化失败返回-1 
		if(file_p->Init()){
			return -1;
		}
	    std::cout<<"lidar init success"<<std::endl;
		return 0;
    };
    //数据采集
    int Lidar::DataCollection(){
        int len=0;
		//等待UDP数据
		if(( len = recvfrom(server_sockfd_,data_raw_,BUFFSIZE,0,(struct sockaddr *)&remote_addr_,&sin_size_) )<0)
		{
			perror("recvfrom error"); 
			return -1;
		}
		return 0;
    };
	int Lidar::DataProcessing(){
		//提取每个通道的入射角Azimuth 共12个通道的数据 提取每个通道的16组点云数据
		for(int i=0;i<12;i++)
		{
			data_process_->lidar_azimuth[i]=static_cast<float>((data_raw_[100*i+3]<<8)+data_raw_[100*i+2])*0.01;//单位°
			for(int j=0;j<16;j++)
			{
			 	data_process_->lidar_distance[i][j]=static_cast<float>((data_raw_[5+i*100+j*3]<<8)+data_raw_[4+i*100+j*3])*2;//the unit is meters
			}
		}
		//提取该组数据的时间戳
		data_process_->lidar_time_stamp=static_cast<uint32_t>((data_raw_[1203]<<24)+(data_raw_[1202]<<16)+(data_raw_[1201]<<8)+data_raw_[1200]);
		++data_process_->lidar_number;//LIDAR数据序列号
		return 0;
	}
	//数据储存
	int Lidar::DataStorage(){
		int ret=0;
		if(( ret=write(file_p->GetFd(),data_process_,sizeof(lidar_data) ))==-1){
			return -1;
		}
		return 0;
	}

    struct Lidar::lidar_data{
        float lidar_azimuth[12]; //12个通道的入射角度，详情见datasheet 
	    float lidar_distance[12][16];//点云距离数据//12*16个距离点，对应的入射角度由Vertical_table和Azimuth决定
	    uint32_t lidar_time_stamp=0;//激光HOT时间 us
	    float lidar_number=0;//LIDAR数据序列号
    };
    int Lidar::Run(){
		while(1){
			if(DataCollection()||DataProcessing()||DataStorage()){
				return -1;
			}
		}
		return 0;
	}





}
