#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include "EtherCAT_Master.h"

#ifndef _EL2889_H_
#define _EL2889_H_

struct EL2889_Domain_Offset{
    unsigned int channel1;
    unsigned int channel1_offset;

    unsigned int channel2;
    unsigned int channel2_offset;

    unsigned int channel3;
    unsigned int channel3_offset;

    unsigned int channel4;
    unsigned int channel4_offset;

    unsigned int channel5;
    unsigned int channel5_offset;

    unsigned int channel6;
    unsigned int channel6_offset;

    unsigned int channel7;
    unsigned int channel7_offset;

    unsigned int channel8;
    unsigned int channel8_offset;

    unsigned int channel9;
    unsigned int channel9_offset;

    unsigned int channel10;
    unsigned int channel10_offset;

    unsigned int channel11;
    unsigned int channel11_offset;

    unsigned int channel12;
    unsigned int channel12_offset;

    unsigned int channel13;
    unsigned int channel13_offset;

    unsigned int channel14;
    unsigned int channel14_offset;

    unsigned int channel15;
    unsigned int channel15_offset;

    unsigned int channel16;
    unsigned int channel16_offset;
};

struct EL2889_Data
{
    // 定义每一个通道的控制量
    bool channel1;
    bool channel2;
    bool channel3;
    bool channel4;
    bool channel5;
    bool channel6;
    bool channel7;
    bool channel8;
    bool channel9;
    bool channel10;
    bool channel11;
    bool channel12;
    bool channel13;
    bool channel14;
    bool channel15;
    bool channel16;
};


// 定义EL2889的从站信息结构体
struct EL2889
{
  uint16_t alias;        // 定义EL2889输出模块在EtherCAT总线上的位置，总站位置
  uint16_t position;     // 定义EL2889输出模块在EtherCAT总线上的位置，从站位置
  uint32_t vendor_id;    // 定义EL2889输出模块的厂家标识
  uint32_t product_code; // 定义EL2889输出模块的产品标识

  ec_domain_t *domain;            // 域
  ec_domain_state_t domain_state; // 域状态

  ec_slave_config_t *slave_config;            // 从站配置，这里只有一台迈信伺服
  ec_slave_config_state_t slave_config_state; // 从站配置状态

  uint8_t *domain_pd;                            // Process Data 过程数据
  struct EL2889_Domain_Offset domain_offset;     // 定义用于保存domain访问偏移量的结构体

  struct EL2889_Data data;                       // 定义用于EL2889各个通道的控制变量

  ec_pdo_entry_reg_t *domain_regs;
  ec_sync_info_t *syncs;
};

void Init_EL2889(struct EtherCAT_Master *Master, struct EL2889 *Device, uint16_t Alias, uint16_t Position, uint32_t Vendor_id, uint32_t Product_code);
void Get_PD_EL2889(struct EL2889 *Device);
void Read_Data_EL2889(struct EL2889 *Device);
void Write_Data_EL2889(struct EL2889 *Device);
ec_pdo_entry_reg_t *Get_PDO_Entry_Reg_EL2889(struct EL2889 *Device);
ec_sync_info_t *Get_Sync_Info_EL2889(struct EL2889 *Device);


#endif