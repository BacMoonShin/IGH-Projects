/*
 * @Author: your name
 * @Date: 2024-09-27 16:06:42
 * @LastEditors: your name
 * @LastEditTime: 2024-09-27 18:09:28
 * @Description:
 * @FilePath: /EtherCAT_CXX/EtherCAT_Slave.h
 */
#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>


#ifndef _ETHERCAT_SLAVE_H_
#define _ETHERCAT_SLAVE_H_

class EtherCAT_Master;

class EtherCAT_Slave_Base
{
  public:
    uint16_t alias;        // 定义从站设备在EtherCAT总线上的位置，总站位置
    uint16_t position;     // 定义从站设备在EtherCAT总线上的位置，从站位置
    uint32_t vendor_id;    // 定义从站设备的厂家标识
    uint32_t product_id;   // 定义从站设备的产品标识

    uint8_t *domain_pd;                            // Process Data 过程数据
    ec_domain_t *domain;            // 域
    ec_domain_state_t domain_state; // 域状态

    ec_slave_config_t *slave_config;            // 从站配置
    ec_slave_config_state_t slave_config_state; // 从站配置状态

    ec_pdo_entry_reg_t *domain_regs;
    ec_sync_info_t *syncs;
    virtual void Get_PDO_Entry_Reg() = 0;
    virtual void Get_Sync_Info() = 0;

    virtual void Add_Slave_To_Master(EtherCAT_Master& Master) = 0;

    virtual void Read_Data() = 0;
    virtual void Write_Data() = 0;
};

#endif