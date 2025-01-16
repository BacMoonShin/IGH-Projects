#ifndef _ETHERCAT_SLAVE_H_
#define _ETHERCAT_SLAVE_H_

#include "EtherCAT_Global.h"

// CoE对象字典
#define RXPDO 0x1600   // RxPDO的映射表在对象字典中的索引位置
#define TXPDO 0x1A00   // TxPDO的映射表在对象字典中的索引位置

/*CiA 402数据对象(Data Object)*/
#define ANALOG_INPUT 0x2205, 0x01                // 模拟量输入

#define ERROR_CODE 0x603F, 0x00                  // 错误码
#define CTRL_WORD 0x6040, 0x00                   // 控制字
#define STATUS_WORD 0x6041, 0x00                 // 状态字
#define OPERATION_MODE 0x6060, 0x00              // 设定运行模式
#define MODE_DISPLAY 0x6061, 0x00                // 当前运行模式
#define CURRENT_POSITION 0x6064, 0x00            // 当前位置
#define CURRENT_VELOCITY 0x606C, 0x00            // 当前速度
#define TARGET_TORQUE 0x6071, 0x00               // 目标转矩
#define MAX_CURRENT 0x6073, 0x00                 // 最大电流
#define CURRENT_TORQUE 0x6077, 0x00              // 当前转矩
#define TARGET_POSITION 0x607A, 0x00             // 目标位置
#define MAX_SPEED 0x607F, 0x00                   // 最大转速
#define MOTOR_RESOLUTION 0x6091, 0x01            // 电机分辨率
#define AXIS_RESOLUTION 0x6091, 0x02             // 负载轴分辨率
#define SERVO_PROBE 0x60B8, 0x00                 // 探针功能
#define PROBE_STATUS 0x60B9, 0x00                // 探针状态
#define PROBE1_RISE_EDGE_POSITION 0x60BA, 0x00   // 探针1上升沿位置反馈
#define PROBE2_RISE_EDGE_POSITION 0x60BC, 0x00   // 探针2上升沿位置反馈
#define POSITION_DEVIATION 0x60F4, 0x00          // 位置偏差
#define DI_STATUS 0x60FD, 0x00                   // DI状态
#define DO_STATUS 0x60FE, 0x01                   // DO输出
#define TARGET_VELOCITY 0x60FF, 0x00             // 目标速度

// CIA402 CANOpen over EtherCAT 驱动器状态
enum DRIVERSTATE
{
    dsNotReadyToSwitchOn = 0, // 初始化 未完成状态
    dsSwitchOnDisabled,       // 初始化 完成状态
    dsReadyToSwitchOn,        // 主电路电源OFF状态
    dsSwitchedOn,             // 伺服OFF/伺服准备
    dsOperationEnabled,       // 伺服使能
    dsQuickStopActive,        // 快速停机状态
    dsFaultReactionActive,    // 异常（报警）判断
    dsFault                   // 异常（报警）状态
};

// CIA402 CANOpen over EtherCAT 驱动器运行模式
enum DRIVERMODE
{
    PP = 1,           // 轮廓位置模式
    PV = 3,           // 轮廓速度模式
    PT = 4,           // 轮廓转矩模式
    HM = 6,           // 回零模式
    CSP = 8,          // 周期同步位置模式
    CSV = 9,          // 周期同步速度模式
    CST = 10          // 周期同步转矩模式
};

struct ec_sdo_config_reg{
    uint16_t index;
    uint8_t subindex;
    void * data;
    size_t data_size;
    uint32_t * abort_code;
};
typedef ec_sdo_config_reg ec_sdo_config_reg_t;

class EtherCAT_Master;

class EtherCAT_Slave_Base
{
  public:
    uint16_t alias;        // 定义从站设备在EtherCAT总线上的位置，总站位置
    uint16_t position;     // 定义从站设备在EtherCAT总线上的位置，从站位置
    uint32_t vendor_id;    // 定义从站设备的厂家标识
    uint32_t product_code;   // 定义从站设备的产品标识
    uint32_t assign_activate_word; // 定义从站用于配置DC的控制字

    uint8_t *domain_pd;             // Process Data 过程数据，用于配合指针偏移量读取数据
    ec_domain_t *domain;            // 域
    ec_domain_state_t domain_state; // 域状态

    ec_slave_config_t *slave_config;            // 从站配置
    ec_slave_config_state_t slave_config_state; // 从站配置状态

    const ec_pdo_entry_reg_t *domain_regs;
    const ec_sdo_config_reg_t *sdo_config_regs;
    const ec_sync_info_t *syncs;

    virtual void Get_PDO_Entry_Reg() = 0;
    virtual void Get_Sync_Info() = 0;
    virtual void Get_SDO_Config_Reg() = 0;
    virtual void Add_Slave_To_Master(EtherCAT_Master& _Master) = 0;
    virtual void Read_Data() = 0;
    virtual void Write_Data() = 0;
    virtual void Slave_Exit() = 0;
};

#endif