#include "HC_SV660N.h"

ec_pdo_entry_info_t HC_SV660N::HC_SV660N_pdo_entries[16] = {/*RxPDO 0x1702*/
                                                            {CTRL_WORD, 16},
                                                            {TARGET_POSITION, 32},
                                                            {TARGET_VELOCITY, 32},
                                                            {TARGET_TORQUE, 16},
                                                            {OPERATION_MODE, 8},
                                                            {SERVO_PROBE, 16},
                                                            {MAX_SPEED, 32},
                                                            /* TxPDO 0x1B01 */
                                                            {ERROR_CODE, 16},
                                                            {STATUS_WORD, 16},
                                                            {CURRENT_POSITION, 32},
                                                            {CURRENT_TORQUE, 16},
                                                            {POSITION_DEVIATION, 32},
                                                            {PROBE_STATUS, 16},
                                                            {PROBE1_RISE_EDGE_POSITION, 32},
                                                            {PROBE2_RISE_EDGE_POSITION, 32},
                                                            {DI_STATUS, 32},
                                                          };
ec_pdo_info_t HC_SV660N::HC_SV660N_pdos[2] = {// RxPDO
                                              {0x1702, 7, HC_SV660N_pdo_entries + 0},
                                               // TxPDO
                                              {0x1b01, 9, HC_SV660N_pdo_entries + 7},
                                              };

HC_SV660N::HC_SV660N(){}
HC_SV660N::~HC_SV660N(){}

HC_SV660N::HC_SV660N(EtherCAT_Master& Master, const uint16_t& Alias, const uint16_t& Position, const struct HC_SV660N_Motor& Motor){
    alias = Alias;
    position = Position;
    motor = Motor;
    vendor_id = HC_SV660N_VENDOR_ID;
    product_code = HC_SV660N_PRODUCT_CODE;
    assign_activate_word = HC_SV660N_ASSIGN_ACTIVATE_WORD;

    Get_PDO_Entry_Reg();
    Get_Sync_Info();
    Get_SDO_Config_Reg();
    Add_Slave_To_Master(Master);

    HC_SV660N_sdo_config_regs[0] = (ec_sdo_config_reg_t){0};
}

HC_SV660N::HC_SV660N(EtherCAT_Master& Master, const uint16_t& Alias, const uint16_t& Position, const struct HC_SV660N_Motor& Motor, const struct HC_SV660N_Sdo_Data& SDO_Data){
    alias = Alias;
    position = Position;
    motor = Motor;
    sdo_data = SDO_Data;
    vendor_id = HC_SV660N_VENDOR_ID;
    product_code = HC_SV660N_PRODUCT_CODE;
    assign_activate_word = 0x0300;

    Get_PDO_Entry_Reg();
    Get_Sync_Info();
    Get_SDO_Config_Reg();
    Add_Slave_To_Master(Master);
}

void HC_SV660N::Check_Motor_State(){
    static int motorLastState = -1;
    if(data.drive_state != motorLastState){
        switch(data.drive_state){
            case dsNotReadyToSwitchOn:
                ETHERCAT_INFO("*SV660N: dsNotReadyToSwitchOn*");
                break;
            case dsSwitchOnDisabled:
                ETHERCAT_INFO("*SV660N: dsSwitchOnDisabled*");
                break;
            case dsReadyToSwitchOn:
                ETHERCAT_INFO("*SV660N: dsReadyToSwitchOn*");
                break;
            case dsSwitchedOn:
                ETHERCAT_INFO("*SV660N: dsSwitchedOn*");
                break;
            case dsOperationEnabled:
                ETHERCAT_INFO("*SV660N: dsOperationEnabled*");
                break;
            case dsQuickStopActive:
                ETHERCAT_INFO("*SV660N: dsQuickStopActive*");
                break;
            case dsFaultReactionActive:
                ETHERCAT_INFO("*SV660N: dsFaultReactionActive*");
                break;
            case dsFault:
                ETHERCAT_INFO("*SV660N: dsFault*");
                break;
        }
        motorLastState = data.drive_state;
    }
}

void HC_SV660N::Reset_Motor_State(){
    switch(data.drive_state){
            case dsReadyToSwitchOn:
            case dsSwitchedOn:
            case dsOperationEnabled:
                domain_data.RxPDO.ctrl_word = 0x0000;
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, domain_data.RxPDO.ctrl_word);
                break;
            case dsFaultReactionActive:
            case dsFault:
                domain_data.RxPDO.ctrl_word = 0x0080;
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, domain_data.RxPDO.ctrl_word);
                break;
            case dsQuickStopActive:
                break;
            case dsSwitchOnDisabled:
                ETHERCAT_MESSAGE("*Reset SV660N state to dsSwitchOnDisabled*");
                data.reset_busy = false;
                data.quickstop_busy = false;
                data.position_move_enable = false;
                break;
            case dsNotReadyToSwitchOn:
                break;
        }
}

void HC_SV660N::Enable_Motor(const DRIVEMODE DriveMode){
    data.drive_mode = DriveMode;
    switch (data.drive_state){
            case dsNotReadyToSwitchOn:
                ETHERCAT_WARNING("*Can not enable SV660N from dsNotReadyToSwitchOn state*");
                break;
            case dsSwitchOnDisabled:
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, 0x0006);
                // ETHERCAT_MESSAGE("*Change SV660N state to dsReadyToSwitchOn*");
                break;
            case dsReadyToSwitchOn:
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, 0x0007);
                // ETHERCAT_MESSAGE("*Change SV660N state to dsSwitchedOn*");
                break;
            case dsSwitchedOn:
                // 设置运行模式
                domain_data.RxPDO.operation_mode = data.drive_mode;
                EC_WRITE_U8(domain_pd + domain_offset.RxPDO.operation_mode, domain_data.RxPDO.operation_mode);
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, 0x000f); // enable operation
                // ETHERCAT_MESSAGE("*Change SV660N state to dsOperationEnabled*");
                domain_data.RxPDO.target_position = domain_data.TxPDO.current_position; // 将当前位置复制给目标位置，防止使能后电机震动
                if(data.get_origin_point == true)
                {
                    data.move_origin_point = domain_data.TxPDO.current_position;
                    data.get_origin_point == false;
                }
                EC_WRITE_S32(domain_pd + domain_offset.RxPDO.target_position, domain_data.RxPDO.target_position);
                break;
            case dsOperationEnabled:
                ETHERCAT_MESSAGE("*SV660N: Success to enable*");
                data.power_busy = false;
                data.position_move_enable = true;
                break;
            default:
                ETHERCAT_WARNING("*Can not enable SV660N*");
                data.reset_busy = true;
        }
}

void HC_SV660N::Motor_Move_To_Home(){
    // 开始回零
    if (domain_data.TxPDO.current_position - data.move_origin_point > 0)
    {
        domain_data.RxPDO.target_position = domain_data.TxPDO.current_position - (int32_t)HOME_STEP;
        if (domain_data.RxPDO.target_position - data.move_origin_point< 0)
        {
            domain_data.RxPDO.target_position = data.move_origin_point;
        }
    }
    else if (domain_data.TxPDO.current_position - data.move_origin_point< 0)
        {
            domain_data.RxPDO.target_position = domain_data.TxPDO.current_position + (int32_t)HOME_STEP;
            if (domain_data.RxPDO.target_position - data.move_origin_point> 0)
            {
                domain_data.RxPDO.target_position = data.move_origin_point;
            }
        }
    else if (domain_data.TxPDO.current_position == data.move_origin_point)
        {
            domain_data.RxPDO.target_position = data.move_origin_point;
            data.home_busy = false; // 回零结束
            data.position_move_enable = false;
        }
    EC_WRITE_S32(domain_pd + domain_offset.RxPDO.target_position, domain_data.RxPDO.target_position);
}

void HC_SV660N::Slave_Exit(){
    Reset_Motor_State();
}

void HC_SV660N::Get_PDO_Entry_Reg(){
    HC_SV660N_domain_regs[0] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, CTRL_WORD, &domain_offset.RxPDO.ctrl_word, NULL};
    HC_SV660N_domain_regs[1] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, TARGET_POSITION, &domain_offset.RxPDO.target_position, NULL};
    HC_SV660N_domain_regs[2] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, TARGET_VELOCITY, &domain_offset.RxPDO.target_velocity, NULL};
    HC_SV660N_domain_regs[3] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, TARGET_TORQUE, &domain_offset.RxPDO.target_torque, NULL};
    HC_SV660N_domain_regs[4] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, OPERATION_MODE, &domain_offset.RxPDO.operation_mode, NULL};
    HC_SV660N_domain_regs[5] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, SERVO_PROBE, &domain_offset.RxPDO.servo_probe, NULL};
    HC_SV660N_domain_regs[6] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, MAX_SPEED, &domain_offset.RxPDO.max_speed, NULL};
    HC_SV660N_domain_regs[7] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, ERROR_CODE, &domain_offset.TxPDO.error_code, NULL};
    HC_SV660N_domain_regs[8] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, STATUS_WORD, &domain_offset.TxPDO.status_word, NULL};
    HC_SV660N_domain_regs[9] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, CURRENT_POSITION, &domain_offset.TxPDO.current_position, NULL};
    HC_SV660N_domain_regs[10] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, CURRENT_TORQUE, &domain_offset.TxPDO.current_torque, NULL};
    HC_SV660N_domain_regs[11] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, POSITION_DEVIATION, &domain_offset.TxPDO.position_deviation, NULL};
    HC_SV660N_domain_regs[12] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, PROBE_STATUS, &domain_offset.TxPDO.probe_status, NULL};
    HC_SV660N_domain_regs[13] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, PROBE1_RISE_EDGE_POSITION, &domain_offset.TxPDO.probe1_rise_edge_position, NULL};
    HC_SV660N_domain_regs[14] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, PROBE2_RISE_EDGE_POSITION, &domain_offset.TxPDO.probe2_rise_edge_position, NULL};
    HC_SV660N_domain_regs[15] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, DI_STATUS, &domain_offset.TxPDO.di_status, NULL};
    HC_SV660N_domain_regs[16] =
    (ec_pdo_entry_reg_t){};

    domain_regs = HC_SV660N_domain_regs;
}

void HC_SV660N::Get_Sync_Info(){
    HC_SV660N_sync2_pdos[0] = HC_SV660N_pdos[0];
    HC_SV660N_sync3_pdos[0] = HC_SV660N_pdos[1];
    HC_SV660N_syncs[0] = (ec_sync_info_t){0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE};
    HC_SV660N_syncs[1] = (ec_sync_info_t){1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE};
    HC_SV660N_syncs[2] = (ec_sync_info_t){2, EC_DIR_OUTPUT, 1, HC_SV660N_sync2_pdos, EC_WD_ENABLE};
    HC_SV660N_syncs[3] = (ec_sync_info_t){3, EC_DIR_INPUT, 1, HC_SV660N_sync3_pdos, EC_WD_DISABLE};
    HC_SV660N_syncs[4] = (ec_sync_info_t){0xff};
    syncs = HC_SV660N_syncs;
}

void HC_SV660N::Get_SDO_Config_Reg(){
    HC_SV660N_sdo_config_regs[0] =
    (ec_sdo_config_reg_t){MOTOR_RESOLUTION, (uint8_t*)&sdo_data.motor_resolution_0x6061_0x01, 4, &data.abort_code};
    HC_SV660N_sdo_config_regs[1] =
    (ec_sdo_config_reg_t){AXIS_RESOLUTION, (uint8_t*)&sdo_data.axis_resolution_0x6061_0x02, 4, &data.abort_code};
    HC_SV660N_sdo_config_regs[2] =
    (ec_sdo_config_reg_t){0};

    sdo_config_regs = HC_SV660N_sdo_config_regs;
}

void HC_SV660N::Add_Slave_To_Master(EtherCAT_Master& Master){
    Master.Slaves.push_back((EtherCAT_Slave_Base *)this);
}

void HC_SV660N::Read_Data(){
    domain_data.TxPDO.error_code =
        EC_READ_U16(domain_pd + domain_offset.TxPDO.error_code);
    domain_data.TxPDO.status_word =
        EC_READ_U16(domain_pd + domain_offset.TxPDO.status_word);
    domain_data.TxPDO.current_position =
        EC_READ_S32(domain_pd + domain_offset.TxPDO.current_position);
    domain_data.TxPDO.current_torque =
        EC_READ_S16(domain_pd + domain_offset.TxPDO.current_torque);
    domain_data.TxPDO.position_deviation =
        EC_READ_S32(domain_pd + domain_offset.TxPDO.position_deviation);
    domain_data.TxPDO.probe_status =
        EC_READ_U16(domain_pd + domain_offset.TxPDO.probe_status);
    domain_data.TxPDO.probe1_rise_edge_position =
        EC_READ_S32(domain_pd + domain_offset.TxPDO.probe1_rise_edge_position);
    domain_data.TxPDO.probe2_rise_edge_position =
        EC_READ_S32(domain_pd + domain_offset.TxPDO.probe2_rise_edge_position);
    domain_data.TxPDO.di_status =
        EC_READ_U32(domain_pd + domain_offset.TxPDO.di_status);


    data.actual_torque = ((double)(domain_data.TxPDO.current_torque) / (double)(0x1000)) * motor.rated_torque;
    // DS402 CANOpen over EtherCAT status machine
    // 获取当前驱动器的驱动状态XZ
    if ((domain_data.TxPDO.status_word & 0x004F) == 0x0000)
        data.drive_state = dsNotReadyToSwitchOn;   //  初始化 未完成状态
    else if ((domain_data.TxPDO.status_word & 0x004F) == 0x0040)
        data.drive_state = dsSwitchOnDisabled;     //  初始化完成 伺服无故障状态
    else if ((domain_data.TxPDO.status_word & 0x006F) == 0x0021)
        data.drive_state = dsReadyToSwitchOn;      //  伺服准备好状态
    else if ((domain_data.TxPDO.status_word & 0x006F) == 0x0023)
        data.drive_state = dsSwitchedOn;           //  伺服启动，等待打开伺服使能
    else if ((domain_data.TxPDO.status_word & 0x006F) == 0x0027)
        data.drive_state = dsOperationEnabled;     //  伺服使能打开，伺服正常运行状态
    else if ((domain_data.TxPDO.status_word & 0x006F) == 0x0007)
        data.drive_state = dsQuickStopActive;      //  快速停机有效
    else if ((domain_data.TxPDO.status_word & 0x004F) == 0x000F)
        data.drive_state = dsFaultReactionActive;  //  故障停机，故障反应有效
    else if ((domain_data.TxPDO.status_word & 0x004F) == 0x0008)
        data.drive_state = dsFault;                //  故障状态
}

void HC_SV660N::Write_Data(){
    EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, domain_data.RxPDO.ctrl_word);
    EC_WRITE_S32(domain_pd + domain_offset.RxPDO.target_position, domain_data.RxPDO.target_position);
    EC_WRITE_S32(domain_pd + domain_offset.RxPDO.target_velocity, domain_data.RxPDO.target_velocity);
    EC_WRITE_S16(domain_pd + domain_offset.RxPDO.target_torque, domain_data.RxPDO.target_torque);
    EC_WRITE_U8(domain_pd + domain_offset.RxPDO.operation_mode, domain_data.RxPDO.operation_mode);
    EC_WRITE_U16(domain_pd + domain_offset.RxPDO.servo_probe, domain_data.RxPDO.servo_probe);
    EC_WRITE_U32(domain_pd + domain_offset.RxPDO.max_speed, domain_data.RxPDO.max_speed);
}