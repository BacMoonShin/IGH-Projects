#include "EtherCAT_Master.h"
#include "EtherCAT_Slave.h"

#ifndef _HC_SV660N_H_
#define _HC_SV660N_H_

#define HC_SV660N_VENDOR_ID 0x00100000
#define HC_SV660N_PRODUCT_CODE 0x000c010d
#define HC_SV660N_ASSIGN_ACTIVATE_WORD 0x0300

struct HC_SV660N_Domain_Offset_RxPDO
{
    /* RxPDO */
    uint32_t ctrl_word;           // 控制字
    uint32_t target_position;     // 目标位置
    uint32_t target_velocity;     // 目标速度
    uint32_t target_torque;       // 目标转矩
    uint32_t operation_mode;      // 运行模式选择，如位置模式，速度模式等
    uint32_t servo_probe;         // 探针功能
    uint32_t max_speed;           // 最大转速
};

struct HC_SV660N_Domain_Offset_TxPDO
{
    /* TxPDO */
    uint32_t error_code;          // 错误码
    uint32_t status_word;         // 状态字
    uint32_t current_position;    // 位置反馈
    uint32_t current_torque;      // 转矩反馈
    uint32_t position_deviation;  // 位置偏差
    uint32_t probe_status;         // 探针状态
    uint32_t probe1_rise_edge_position;  // 探针1上升沿位置反馈
    uint32_t probe2_rise_edge_position;  // 探针2上升沿位置反馈
    uint32_t di_status;            // DI状态
};

struct HC_SV660N_Domain_Offset
{
  /* RxPDO */
    struct HC_SV660N_Domain_Offset_RxPDO RxPDO;

  /* TxPDO */
    struct HC_SV660N_Domain_Offset_TxPDO TxPDO;
};

struct HC_SV660N_Domain_Data_RxPDO
{
    /* RxPDO */
    uint16_t ctrl_word;           // 控制字
    int32_t target_position;      // 目标位置
    int32_t target_velocity;      // 目标速度
    int16_t target_torque;        // 目标转矩
    uint8_t operation_mode;       // 运行模式选择，如位置模式，速度模式等
    uint16_t servo_probe;         // 探针功能
    uint32_t max_speed;           // 最大转速
};

struct HC_SV660N_Domain_Data_TxPDO
{
    /* TxPDO */
    uint32_t error_code;           // 错误码
    uint32_t status_word;          // 状态字
    int32_t current_position;      // 位置反馈
    int16_t current_torque;        // 转矩反馈
    int32_t position_deviation;    // 位置偏差
    uint16_t probe_status;         // 探针状态
    int32_t probe1_rise_edge_position;  // 探针1上升沿位置反馈
    int32_t probe2_rise_edge_position;  // 探针2上升沿位置反馈
    uint32_t di_status;            // DI状态
};

struct HC_SV660N_Domain_Data
{
    // 定义伺服驱动的控制数据
    /* RxPDO */
    struct HC_SV660N_Domain_Data_RxPDO RxPDO;

    /* TxPDO */
    struct HC_SV660N_Domain_Data_TxPDO TxPDO;
};

struct HC_SV660N_Sdo_Data{
    uint32_t motor_resolution;
    uint32_t axis_resolution;
};

struct HC_SV660N_Data
{
    // 定义驱动器状态控制有关的变量
    enum DRIVERSTATE drive_state;  // 驱动器运行状态
    enum DRIVEMODE drive_mode;     // 驱动器运行模式
    bool reset_busy = true;
    bool power_busy = true;
    bool quickstop_busy = false;
    bool home_busy = false;
    bool position_move_enable = false;

    // 电机有关变量
    bool get_origin_point = true;
    double actual_torque = 0;
    int32_t move_origin_point;
    // double screw_target_velocity;   // mm/s，丝杠运动速度定义

    // SDO下载中断码
    uint32_t abort_code;
};

struct HC_SV660N_Motor
{
    // 定义电机有关参数
    uint32_t motor_encoder_resolution;
    double rated_torque;
};


class HC_SV660N : public EtherCAT_Slave_Base
{
  public:
    struct HC_SV660N_Domain_Offset domain_offset;     // 定义用于保存domain访问偏移量的结构体
    struct HC_SV660N_Domain_Data domain_data;         // 定义用于HC_SV660N的PDO控制变量
    struct HC_SV660N_Sdo_Data sdo_data;               // 定义HC_SV660N的SDO控制变量
    struct HC_SV660N_Data data;                       // 定义驱动器内部变量
    struct HC_SV660N_Motor motor;                     // 定义用于保存电机参数的结构体

    ec_sdo_config_reg_t HC_SV660N_sdo_config_regs[3];

    ec_pdo_entry_reg_t HC_SV660N_domain_regs[17];

    static ec_pdo_entry_info_t HC_SV660N_pdo_entries[16];
    static ec_pdo_info_t HC_SV660N_pdos[2];

    ec_pdo_info_t HC_SV660N_sync2_pdos[1];
    ec_pdo_info_t HC_SV660N_sync3_pdos[1];
    ec_sync_info_t HC_SV660N_syncs[5];

    HC_SV660N();
    ~HC_SV660N();
    HC_SV660N(EtherCAT_Master& Master, const uint16_t& Alias, const uint16_t& Position, const struct HC_SV660N_Motor& Motor);
    HC_SV660N(EtherCAT_Master& Master, const uint16_t& Alias, const uint16_t& Position, const struct HC_SV660N_Motor& Motor, const struct HC_SV660N_Sdo_Data& SDO_Data);

    void Check_Motor_State();
    void Reset_Motor_State();
    void Enable_Motor(const DRIVEMODE DriveMode);
    void Motor_Move_To_Home();

    virtual void Get_PDO_Entry_Reg() override;
    virtual void Get_Sync_Info() override;
    virtual void Get_SDO_Config_Reg() override;

    virtual void Add_Slave_To_Master(EtherCAT_Master& Master) override;

    virtual void Read_Data() override;
    virtual void Write_Data() override;

    virtual void Slave_Exit() override;
};

#endif