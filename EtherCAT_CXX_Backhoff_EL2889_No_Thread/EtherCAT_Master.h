/*
 * @Author: your name
 * @Date: 2024-09-27 10:20:23
 * @LastEditors: your name
 * @LastEditTime: 2024-09-27 16:42:00
 * @Description:
 * @FilePath: /EtherCAT_CXX/EtherCAT_Master.h
 */
#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include <vector>
#include "EtherCAT_Slave.h"

#ifndef _ETHERCAT_MASTER_H_
#define _ETHERCAT_MASTER_H_

class EtherCAT_Slave_Base;

class EtherCAT_Master
{
  public:
    ec_master_t *master;            // 主站
    ec_master_state_t master_state; // 主站状态

    std::vector<EtherCAT_Slave_Base *> Slaves; // 所有从站设备的列表

    void EtherCAT_Master_Config();
    void EtherCAT_Master_Deactivate();

  private:
    void EtherCAT_Master_Init();
    void EtherCAT_Config_Slave(EtherCAT_Slave_Base * Slave, int Slave_Index);
    void EtherCAT_Master_Activate();
    void EtherCAT_Get_PD_For_Slave(EtherCAT_Slave_Base * Slave, int Slave_Index);
};

#endif