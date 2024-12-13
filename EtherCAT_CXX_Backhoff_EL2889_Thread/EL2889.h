#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include <vector>
#include "EtherCAT_Slave.h"
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

class EL2889 : public EtherCAT_Slave_Base
{
  public:
    struct EL2889_Domain_Offset domain_offset;     // 定义用于保存domain访问偏移量的结构体
    struct EL2889_Data data;                       // 定义用于EL2889各个通道的控制变量

    ec_pdo_entry_reg_t EL2889_domain_regs[17];

    static ec_pdo_entry_info_t EL2889_pdo_entries[16];
    static ec_pdo_info_t EL2889_pdos[16];

    ec_pdo_info_t EL2889_sync0_pdos[8];
    ec_pdo_info_t EL2889_sync1_pdos[8];
    ec_sync_info_t EL2889_syncs[3];

    EL2889(EtherCAT_Master& Master, uint16_t Alias, uint16_t Position);
    virtual void Get_PDO_Entry_Reg() override;
    virtual void Get_Sync_Info() override;

    virtual void Add_Slave_To_Master(EtherCAT_Master& Master) override;

    virtual void Read_Data() override;
    virtual void Write_Data() override;
};

#endif