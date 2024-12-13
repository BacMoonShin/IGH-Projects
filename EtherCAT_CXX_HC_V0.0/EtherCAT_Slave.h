/*
 * @Author: your name
 * @Date: 2024-09-27 16:06:42
 * @LastEditors: your name
 * @LastEditTime: 2024-09-27 18:09:28
 * @Description:
 * @FilePath: /EtherCAT_CXX/EtherCAT_Slave.h
 */
#include <ecrt.h>
#include <stdint.h>

#ifndef _ETHERCAT_SLAVE_H_
#define _ETHERCAT_SLAVE_H_

// CoE对象字典
#define RXPDO 0x1600   // RxPDO的映射表在对象字典中的索引位置
#define TXPDO 0x1A00   // TxPDO的映射表在对象字典中的索引位置

/*CiA 402数据对象(Data Object)*/
#define ERROR_CODE 0x603F                 // 错误码
#define CTRL_WORD 0x6040                  // 控制字
#define STATUS_WORD 0x6041                // 状态字
#define OPERATION_MODE 0x6060             // 设定运行模式
#define MODE_DISPLAY 0x6061               // 当前运行模式
#define CURRENT_POSITION 0x6064           // 当前位置
#define CURRENT_VELOCITY 0x606C           // 当前速度
#define TARGET_TORQUE 0x6071              // 目标转矩
#define CURRENT_TORQUE 0x6077             // 当前转矩
#define TARGET_POSITION 0x607A            // 目标位置
#define MAX_SPEED 0x607F                  // 最大转速
#define SERVO_PROBE 0x60B8                // 探针功能
#define PROBE_STATUS 0x60B9               // 探针状态
#define PROBE1_RISE_EDGE_POSITION 0x60BA  // 探针1上升沿位置反馈
#define PROBE2_RISE_EDGE_POSITION 0x60BC  // 探针2上升沿位置反馈
#define POSITION_DEVIATION 0x60F4         // 位置偏差
#define DI_STATUS 0x60FD                  // DI状态
#define TARGET_VELOCITY 0x60FF            // 目标速度

// DS402 CANOpen over EtherCAT 驱动器状态
enum DRIVERSTATE
{
    dsNotReadyToSwitchOn = 0, // 初始化 未完成状态
    dsSwitchOnDisabled,       // 初始化 完成状态
    dsReadyToSwitchOn,        // 主电路电源OFF状态
    dsSwitchedOn,             // 伺服OFF/伺服准备
    dsOperationEnabled,       // 伺服ON
    dsQuickStopActive,        // 即停止
    dsFaultReactionActive,    // 异常（报警）判断
    dsFault                   // 异常（报警）状态
};

class EtherCAT_Master;

class EtherCAT_Slave_Base
{
  public:
    uint16_t alias;        // 定义从站设备在EtherCAT总线上的位置，总站位置
    uint16_t position;     // 定义从站设备在EtherCAT总线上的位置，从站位置
    uint32_t vendor_id;    // 定义从站设备的厂家标识
    uint32_t product_code;   // 定义从站设备的产品标识

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