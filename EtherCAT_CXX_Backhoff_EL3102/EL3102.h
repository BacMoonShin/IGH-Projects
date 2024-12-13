#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include "EtherCAT_Master.h"

#ifndef _EL3102_H_
#define _EL3102_H_

struct EL3102_Domain_Offset{
    uint32_t channal1_status_offset;
    uint32_t channal1_value_offset;
    uint32_t channal2_status_offset;
    uint32_t channal2_value_offset;
};

struct EL3102_Data
{
    uint16_t channal1_status;
    int32_t channal1_value;
    uint16_t channal2_status;
    int32_t channal2_value;
};


// 定义EL3102的从站信息结构体
struct EL3102
{
  uint16_t alias;        // 定义EL3102输出模块在EtherCAT总线上的位置，总站位置
  uint16_t position;     // 定义EL3102输出模块在EtherCAT总线上的位置，从站位置
  uint32_t vendor_id;    // 定义EL3102输出模块的厂家标识
  uint32_t product_code; // 定义EL3102输出模块的产品标识

  ec_domain_t *domain;            // 域
  ec_domain_state_t domain_state; // 域状态

  ec_slave_config_t *slave_config;            // 从站配置，这里只有一台迈信伺服
  ec_slave_config_state_t slave_config_state; // 从站配置状态

  uint8_t *domain_pd;                            // Process Data 过程数据
  struct EL3102_Domain_Offset domain_offset;     // 定义用于保存domain访问偏移量的结构体

  struct EL3102_Data data;                       // 定义用于EL3102各个通道的控制变量

  ec_pdo_entry_reg_t *domain_regs;
  ec_sync_info_t *syncs;
};

void Init_EL3102(struct EtherCAT_Master *Master, struct EL3102 *Device, uint16_t Alias, uint16_t Position, uint32_t Vendor_id, uint32_t Product_code);
void Get_PD_EL3102(struct EL3102 *Device);
void Read_Data_EL3102(struct EL3102 *Device);
void Write_Data_EL3102(struct EL3102 *Device);
ec_pdo_entry_reg_t *Get_PDO_Entry_Reg_EL3102(struct EL3102 *Device);
ec_sync_info_t *Get_Sync_Info_EL3102(struct EL3102 *Device);


#endif