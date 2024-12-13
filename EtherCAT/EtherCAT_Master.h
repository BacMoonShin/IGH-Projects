/*
 * @Author: your name
 * @Date: 2024-09-24 16:25:21
 * @LastEditors: your name
 * @LastEditTime: 2024-09-24 18:49:30
 * @Description:
 * @FilePath: \EtherCAT\EtherCAT_Master.h
 */
#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>

#ifndef _ETHERCAT_MASTER_H_
#define _ETHERCAT_MASTER_H_

struct EtherCAT_Master
{
    ec_master_t *master;            // 主站
    ec_master_state_t master_state; // 主站状态

    int slaves_number;              // 从站个数
};

void EtherCAT_Master_Init(struct EtherCAT_Master * Master);
void EtherCAT_Master_Activate(struct EtherCAT_Master * Master);


#endif