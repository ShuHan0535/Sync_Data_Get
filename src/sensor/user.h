#pragma once

//imu中断命令宏
#define IMU_IRQ  _IOR('i',1,int)
//gnss中断命令宏
#define GNSS_IRQ  _IOR('g',1,int)
//camera中断命令宏
#define CAM_IRQ  _IOR('c',1,int)
//memory相关宏定义
#define PAGE_SIZE  ((size_t)getpagesize())
#define PAGE_MASK ((uint64_t) (long)~(PAGE_SIZE - 1))