#include "sensor/camera.h"

namespace sensor{
    Camera::Camera(const char* sensor_name,const int& director_num,std::shared_ptr<memory::MemOpt> &data_addr_ptr,const uint64_t& camera_data_offset):
    SensorAbstraction(sensor_name,director_num),data_addr_ptr_(data_addr_ptr),camera_data_offset_(camera_data_offset),camera_irq_fd_(-1){
        data_process_=new camera_data();
    };
    Camera::~Camera(){
        delete data_process_;
        if(camera_irq_fd_ != -1){
            close(camera_irq_fd_);
        }
        //关闭硬件触发模式 关闭相机
        CameraSetTriggerMode(h_camera_,1);
        CameraUnInit(h_camera_);
    }

    int Camera::Init(){
        //配置camera中断
        if((camera_irq_fd_=open("/dev/CAM_irq",O_RDWR)) == -1){
            perror("imu open:");
            return -1;
        }
        //获取camera储存数据的DDR基地址
        camera_addr_=data_addr_ptr_->GetMemoryPhy(camera_data_offset_);

        //初始化本地储存的文件 初始化失败返回-1 
		if(file_p->Init()){
			return -1;
		}

        //配置SDK相机
        //sdk初始化	0 English 1中文
		CameraSdkInit(1);
        int i_camera_counts = 0;
		//枚举设备，并建立设备列表 目前只有一个设备
		CameraEnumerateDevice(&t_camera_enum_list_,&i_camera_counts);
		printf("iCameraCounts =%d  \n",i_camera_counts);
		if(i_camera_counts == 0){
			return -1;
		}
		
		printf(" %s	%s \n",t_camera_enum_list_.acProductName,t_camera_enum_list_.acFriendlyName);
		
		
		//相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
		int cam_status = CameraInit(&t_camera_enum_list_,-1,-1,&h_camera_);
		printf("CameraInit iStatus =%d \n",cam_status);
		//初始化失败
		if(cam_status!=CAMERA_STATUS_SUCCESS){
			return -1;
		}
		//获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
		CameraGetCapability(h_camera_,&t_capability_);
		printf("CameraGetCapability \n");
		pby_buffer_ = (unsigned char*)malloc(t_capability_.sResolutionRange.iHeightMax*t_capability_.sResolutionRange.iWidthMax*3);		
		//设置图像处理的输出格式
		CameraSetIspOutFormat(h_camera_,CAMERA_MEDIA_TYPE_RGB8);

	    //设置自动曝光
	    /*cam_status =CameraSetAeState(h_camera_,TRUE);
	    if(cam_status!=CAMERA_STATUS_SUCCESS)
	    {
		    printf("[ WARN] Set AeExposure failed %d.\n",cam_status);
			return -1;
	    }
	    //设置自动曝光参数
	    double maxAe=10;
	    cam_status=CameraSetAeExposureRange(h_camera_,0,maxAe);
	    if(cam_status!=CAMERA_STATUS_SUCCESS)
	    {
		    printf("[ WARN] Set AeExposureRange failed %d.\n",iStatus);
			return -1;
	    }*/
        //设置固定曝光
        cam_status =CameraSetAeState(h_camera_,false);
        if(cam_status!=CAMERA_STATUS_SUCCESS)
        {
            printf("[ WARN] Set AeExposure failed %d.\n",cam_status);
            return -1;
        }
        //设置曝光时间
        cam_status=CameraSetExposureTime(h_camera_,1000);
        if(cam_status!=CAMERA_STATUS_SUCCESS)
        {
            printf("[ WARN] Set AeExposureRange failed %d.\n",cam_status);
            return -1;
        }

	    cam_status=CameraSetAeTarget(h_camera_,120);
	    if(cam_status!=CAMERA_STATUS_SUCCESS)
	    {
		    printf("[ WARN] Set AeTarget failed %d.\n",cam_status);
			return -1;
	    }
        //设置外触发模式
        cam_status =CameraSetTriggerMode(h_camera_,2);
        if(cam_status!=CAMERA_STATUS_SUCCESS)
        {
            printf("[ WARN] Set trigger mode failed %d.\n",cam_status);
            return -1;
        }
	    CameraPlay(h_camera_);
	    printf("[ INFO] CameraPlay. \n");
	    CameraSetImageResolution(h_camera_,&t_capability_.pImageSizeDesc[0]);
	    usleep(10000);
        printf("CAM Init Success \n");

        return 0;
    }

    int Camera::DataCollection(){
        //等待camera时间戳数据到来
        ioctl(camera_irq_fd_, CAM_IRQ,1);
        memcpy(data_process_,camera_addr_,8);
        ++data_process_->cam_number;
        return 0;
    }
    
    int Camera::DataProcessing(){
        //camera数据处理
        return 0;
    }
    int Camera::DataStorage(){

        //等待图像数据到来
        std::string pic_name=file_p->GetDirectName()+"/picRAW_"+std::to_string(data_process_->cam_number);
        if(CameraGetImageBuffer(h_camera_,&s_frame_info_,&pby_buffer_,200) == CAMERA_STATUS_SUCCESS)
        {
            CameraImageProcess(h_camera_, pby_buffer_, g_prgb_buffer_,&s_frame_info_);
            //保存图片
            CameraSaveImage(h_camera_, const_cast<char*>(pic_name.data()),g_prgb_buffer_, &s_frame_info_, FILE_BMP, 100);
		    //释放，和CameraGetImageBuffer 配套使用
		    CameraReleaseImageBuffer(h_camera_,pby_buffer_);
		    //采集RAW图
    	    /*CameraSaveImage(hCamera, filename,pbyBuffer, &sFrameInfo, FILE_RAW, 0);
    	    CameraReleaseImageBuffer(hCamera,pbyBuffer);*/
        }
        else{
            //保存一个空文件对齐文件序号
            open(const_cast<char*>(pic_name.data()),O_CREAT);
		    CameraReleaseImageBuffer(h_camera_,pby_buffer_);
    	    printf("timeout \n");
        }
        //保存时间戳数据
        int ret=0;
        if( (ret = write(file_p->GetFd(),data_process_,sizeof(camera_data)) < 0) ){
            perror("camera write:");
            return -1;
        }

        return 0;    
    }

    int Camera::Run(){
        while(RunOK){
			if(DataCollection()||DataProcessing()||DataStorage()){
				return -1;
			}
		}
		return 0;
    }
    struct Camera::camera_data{
        camera_data():cam_number(0){};
        uint32_t cam_s_stamp;
	    uint32_t cam_ns_stamp;
	    uint32_t cam_number;
    };

}