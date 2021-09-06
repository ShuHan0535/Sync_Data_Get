#pragma once

//IMU中断命令宏
#define IMU_IRQ  _IOR('i',1,int)//IMU1命令宏

//memory相关宏定义
#define PAGE_SIZE  ((size_t)getpagesize())
#define PAGE_MASK ((uint64_t) (long)~(PAGE_SIZE - 1))