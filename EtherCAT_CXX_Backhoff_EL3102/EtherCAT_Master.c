#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include "EtherCAT_Master.h"

void EtherCAT_Master_Init(struct EtherCAT_Master * Master){
  // 创建ethercat主站master
    Master->master = ecrt_request_master(0);
    if (!Master->master)
    {
        printf("Failed to create ethercat master!\n");
        exit(EXIT_FAILURE); // 创建失败，退出线程
    }
}

void EtherCAT_Master_Activate(struct EtherCAT_Master * Master){
  // 激活主站master
    printf("Activating master...\n");
    if (ecrt_master_activate(Master->master))
    {
        exit(EXIT_FAILURE); // 激活失败，退出线程
    }
    else
    {
        printf("*Master activated*\n"); // 激活成功
    }
}