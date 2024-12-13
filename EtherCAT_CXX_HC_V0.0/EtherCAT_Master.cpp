/*
 * @Author: your name
 * @Date: 2024-09-27 16:28:29
 * @LastEditors: your name
 * @LastEditTime: 2024-09-27 16:43:52
 * @Description:
 * @FilePath: /EtherCAT_CXX/EtherCAT_Master.cpp
 */
#include <stdio.h>
#include <ecrt.h>
#include <unistd.h>
#include <stdint.h>
#include <vector>
#include "EtherCAT_Master.h"
#include "EtherCAT_Slave.h"

void EtherCAT_Master::EtherCAT_Master_Config(){
    EtherCAT_Master_Init();
    for(int i = 0; i < Slaves.size(); i++){
        EtherCAT_Config_Slave(Slaves[i], i);
    }
    EtherCAT_Master_Activate();
    for(int i = 0; i < Slaves.size(); i++){
        EtherCAT_Get_PD_For_Slave(Slaves[i], i);
    }
}

void EtherCAT_Master::EtherCAT_Master_Deactivate(){
    ecrt_master_deactivate(master);
}

void EtherCAT_Master::EtherCAT_Master_Init(){
    // 创建ethercat主站master
    master = ecrt_request_master(0);
    if (!master)
    {
        printf("Failed to create ethercat master!\n");
        exit(EXIT_FAILURE); // 创建失败，退出线程
    }
}

void EtherCAT_Master::EtherCAT_Config_Slave(EtherCAT_Slave_Base * Slave, int Slave_Index){
    // 创建域domain
    Slave->domain = ecrt_master_create_domain(master);
    if (!Slave->domain)
    {
        printf("Failed to create master domain for slave %d!\n", Slave_Index);
        exit(EXIT_FAILURE); // 创建失败，退出线程
    }
    else
    {
        printf("*Success to create master domain for slave %d*\n", Slave_Index);
    }

    // 配置从站
    if (!(Slave->slave_config = ecrt_master_slave_config(master, Slave->alias, Slave->position, Slave->vendor_id, Slave->product_code)))
    {
        printf("Failed to get slave configuration for slave %d!\n", Slave_Index);
        exit(EXIT_FAILURE); // 配置失败，退出线程
    }
    else
    {
        printf("*Success to get slave configuration for slave %d*\n", Slave_Index);
    }

    // 对从站进行配置PDOs
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(Slave->slave_config, EC_END, Slave->syncs))
    {
        printf("Failed to configure PDOs for slave %d!\n", Slave_Index);
        exit(EXIT_FAILURE); // 配置失败，退出线程
    }
    else
    {
        printf("*Success to configuring PDOs for slave %d*\n", Slave_Index); // 配置成功
    }

    // 注册PDO entry
    if (ecrt_domain_reg_pdo_entry_list(Slave->domain, Slave->domain_regs))
    {
        printf("PDO entry registration failed for slave %d!\n", Slave_Index);
        exit(EXIT_FAILURE); // 注册失败，退出线程
    }
    else
    {
        printf("*Success to configuring PDO entry for slave %d*\n", Slave_Index); // 注册成功
    }
}

void EtherCAT_Master::EtherCAT_Master_Activate(){
    // 激活主站master
    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        exit(EXIT_FAILURE); // 激活失败，退出线程
    }
    else
    {
        printf("*Master activated*\n"); // 激活成功
    }
}

void EtherCAT_Master::EtherCAT_Get_PD_For_Slave(EtherCAT_Slave_Base * Slave, int Slave_Index){
    if (!(Slave->domain_pd = ecrt_domain_data(Slave->domain))){
      printf("Get domain_pd failed for slave %d!\n", Slave_Index);
      exit(EXIT_FAILURE);
    }
    else
    {
        printf("*Success to get domain_pd for slave %d*\n", Slave_Index);
    }
}