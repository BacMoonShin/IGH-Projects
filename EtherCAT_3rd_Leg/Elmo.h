#include "EtherCAT_Master.h"
#include "EtherCAT_Slave.h"

#ifndef _ELMO_H_
#define _ELMO_H_

#define ELMO_VENDOR_ID 0x0000009A
#define ELMO_PRODUCT_CODE 0x00030924
#define ELMO_ASSIGN_ACTIVATE_WORD 0x0300

struct ELMO_Domain_Offset_RxPDO
{
    /* RxPDO */
    uint32_t ctrl_word;           // 控制字
    uint32_t target_position;     // 目标位置
    uint32_t digital_output;      // DO输出
    // uint32_t operation_mode;      // 运动模式
};

struct ELMO_Domain_Offset_TxPDO
{
    /* TxPDO */
    uint32_t status_word;         // 状态字
    uint32_t actual_position;    // 位置反馈
    uint32_t di_status;           // DI状态
    uint32_t torque_input;        // 力传感器信号
    // uint32_t mode_display;        // 运动模式反馈
};

struct ELMO_Domain_Offset
{
  /* RxPDO */
    struct ELMO_Domain_Offset_RxPDO RxPDO;

  /* TxPDO */
    struct ELMO_Domain_Offset_TxPDO TxPDO;
};

struct ELMO_Domain_Data_RxPDO
{
    /* RxPDO */
    uint16_t ctrl_word;           // 控制字
    int32_t target_position;      // 目标位置
    uint32_t digital_output;      // DO输出
    // int8_t operation_mode;        // 运动模式
};

struct ELMO_Domain_Data_TxPDO
{
    /* TxPDO */
    uint32_t status_word;          // 状态字
    int32_t actual_position;      // 位置反馈
    uint32_t di_status;            // DI状态
    // int16_t torque_input;          // 力传感器信号
    // int8_t mode_display;           // 运动模式反馈
};

struct ELMO_Domain_Data
{
    // 定义伺服驱动的控制数据
    /* RxPDO */
    struct ELMO_Domain_Data_RxPDO RxPDO;

    /* TxPDO */
    struct ELMO_Domain_Data_TxPDO TxPDO;
};

struct ELMO_SDO_Data{
    uint32_t gear_ratio_num_0x6091_0x01;
    uint32_t gear_ratio_den_0x6091_0x02;
    uint32_t feed_constant_num_0x6092_0x01;
    uint32_t feed_constant_den_0x6092_0x02;
    uint32_t position_encoder_resolution_num_0x608F_0x01;
    uint32_t position_encoder_resolution_den_0x608F_0x02;
    uint32_t velocity_factor_num_0x6096_0x01;
    uint32_t velocity_factor_den_0x6096_0x02;
    int16_t sensor_selection_code_0x606A_0x00;
    int8_t operation_mode_0x6060_0x00;
};

struct ELMO_Data
{
    // 定义驱动器状态控制有关的变量
    enum DRIVERSTATE drive_state;  // 驱动器运行状态
    enum DRIVERMODE drive_mode;
    bool reset_busy = true;
    bool power_busy = false;
    bool quickstop_busy = false;
    bool home_busy = false;
    bool position_move_enable = false;

    // 电机有关变量
    bool get_origin_point = true;
    int32_t move_origin_point;
    // double screw_target_velocity;   // mm/s，丝杠运动速度定义

    // SDO下载中断码
    uint32_t abort_code;
};

struct ELMO_Motor
{
    // 定义电机有关参数
    uint32_t motor_encoder_resolution;
    double rated_torque;
};


class ELMO : public EtherCAT_Slave_Base
{
  private:
    const uint16_t RxPDO = 0x1600;
    const uint16_t TxPDO = 0x1A00;

    static ec_pdo_entry_info_t ELMO_pdo_entries[11];
    static ec_pdo_info_t ELMO_pdos[4];
    ec_pdo_entry_reg_t ELMO_domain_regs[6];
    ec_sdo_config_reg_t ELMO_sdo_config_regs[11];
    ec_pdo_info_t ELMO_sync2_pdos[1];
    ec_pdo_info_t ELMO_sync3_pdos[1];
    ec_sync_info_t ELMO_syncs[5];

    int motor_last_state = -1;

  public:
    struct ELMO_Domain_Offset domain_offset;     // 定义用于保存domain访问偏移量的结构体
    struct ELMO_Domain_Data domain_data;         // 定义用于ELMO的PDO控制变量
    struct ELMO_SDO_Data sdo_data;               // 定义ELMO的SDO控制变量
    struct ELMO_Data data;                       // 定义驱动器内部变量
    struct ELMO_Motor motor;                     // 定义用于保存电机参数的结构体

    ELMO();
    ~ELMO();
    ELMO(EtherCAT_Master& _Master, const uint16_t& _Alias, const uint16_t& _Position);
    ELMO(EtherCAT_Master& _Master, const uint16_t& _Alias, const uint16_t& _Position, const struct ELMO_SDO_Data& _SDO_Data);
    ELMO(EtherCAT_Master& _Master, const uint16_t& _Alias, const uint16_t& _Position, const struct ELMO_Motor& _Motor);
    ELMO(EtherCAT_Master& _Master, const uint16_t& _Alias, const uint16_t& _Position, const struct ELMO_Motor& _Motor, const struct ELMO_SDO_Data& _SDO_Data);

    void Check_Motor_State(int _DeviceNumber) noexcept;
    void Reset_Motor_State() noexcept;
    void Motor_Quickstop() noexcept;
    void Enable_Motor(const DRIVERMODE _DriveMode) noexcept;
    void Move_Motor_Home() noexcept;
    virtual void Read_Data() noexcept override;
    virtual void Write_Data() noexcept override;
    virtual void Slave_Exit() noexcept override;

  private:
    virtual void Get_PDO_Entry_Reg() noexcept override ;
    virtual void Get_Sync_Info() noexcept override;
    virtual void Get_SDO_Config_Reg() noexcept override;
    virtual void Add_Slave_To_Master(EtherCAT_Master& _Master) noexcept override;
};

#endif