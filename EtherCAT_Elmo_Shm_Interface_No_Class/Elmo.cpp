#include "Elmo.h"

ec_pdo_entry_info_t ELMO::ELMO_pdo_entries[11] = {/*RxPDO 0x1600 */
                                                  {TARGET_POSITION, 32},
                                                  {DO_STATUS, 32},
                                                  {CTRL_WORD, 16},
                                                  /*RxPDO 0x1607 */
                                                  {CTRL_WORD, 16},
                                                  {TARGET_POSITION, 32},
                                                  /* TxPDO 0x1A00 */
                                                  {ACTUAL_POSITION, 32},
                                                  {DI_STATUS, 32},
                                                  {STATUS_WORD, 16},
                                                  /* TxPDO 0x1A07 */
                                                  {STATUS_WORD, 16},
                                                  {ACTUAL_POSITION, 32},
                                                  {ANALOG_INPUT, 16},
                                                  };
ec_pdo_info_t ELMO::ELMO_pdos[4] = {// RxPDO
                                   {0x1600, 3, ELMO_pdo_entries + 0},
                                   {0x1607, 2, ELMO_pdo_entries + 3},
                                   // TxPDO
                                   {0x1A00, 3, ELMO_pdo_entries + 5},
                                   {0x1A07, 3, ELMO_pdo_entries + 8},
                                   };

ELMO::ELMO(){}
ELMO::~ELMO(){}
ELMO::ELMO(EtherCAT_Master& _Master, const uint16_t& _Alias, const uint16_t& _Position){
    alias = _Alias;
    position = _Position;
    vendor_id = ELMO_VENDOR_ID;
    product_code = ELMO_PRODUCT_CODE;
    assign_activate_word = ELMO_ASSIGN_ACTIVATE_WORD;

    Get_PDO_Entry_Reg();
    Get_Sync_Info();
    Get_SDO_Config_Reg();
    Add_Slave_To_Master(_Master);

    ELMO_sdo_config_regs[0] = (ec_sdo_config_reg_t){0};
}

ELMO::ELMO(EtherCAT_Master& _Master, const uint16_t& _Alias, const uint16_t& _Position, const struct ELMO_SDO_Data& _SDO_Data){
    alias = _Alias;
    position = _Position;
    sdo_data = _SDO_Data;
    vendor_id = ELMO_VENDOR_ID;
    product_code = ELMO_PRODUCT_CODE;
    assign_activate_word = ELMO_ASSIGN_ACTIVATE_WORD;

    Get_PDO_Entry_Reg();
    Get_Sync_Info();
    Get_SDO_Config_Reg();
    Add_Slave_To_Master(_Master);
}

ELMO::ELMO(EtherCAT_Master& _Master, const uint16_t& _Alias, const uint16_t& _Position, const struct ELMO_Motor& _Motor){
    alias = _Alias;
    position = _Position;
    motor = _Motor;
    vendor_id = ELMO_VENDOR_ID;
    product_code = ELMO_PRODUCT_CODE;
    assign_activate_word = ELMO_ASSIGN_ACTIVATE_WORD;

    Get_PDO_Entry_Reg();
    Get_Sync_Info();
    Get_SDO_Config_Reg();
    Add_Slave_To_Master(_Master);

    ELMO_sdo_config_regs[0] = (ec_sdo_config_reg_t){0};
}

ELMO::ELMO(EtherCAT_Master& _Master, const uint16_t& _Alias, const uint16_t& _Position, const struct ELMO_Motor& _Motor, const struct ELMO_SDO_Data& _SDO_Data){
    alias = _Alias;
    position = _Position;
    motor = _Motor;
    sdo_data = _SDO_Data;
    vendor_id = ELMO_VENDOR_ID;
    product_code = ELMO_PRODUCT_CODE;
    assign_activate_word = ELMO_ASSIGN_ACTIVATE_WORD;

    Get_PDO_Entry_Reg();
    Get_Sync_Info();
    Get_SDO_Config_Reg();
    Add_Slave_To_Master(_Master);
}

void ELMO::Check_Motor_State(int _DeviceNumber) noexcept{
    if(data.drive_state != data.motor_last_state){
        switch(data.drive_state){
            case dsNotReadyToSwitchOn:
                ETHERCAT_INFO("*ELMO %d: dsNotReadyToSwitchOn*", _DeviceNumber);
                break;
            case dsSwitchOnDisabled:
                ETHERCAT_INFO("*ELMO %d: dsSwitchOnDisabled*", _DeviceNumber);
                break;
            case dsReadyToSwitchOn:
                ETHERCAT_INFO("*ELMO %d: dsReadyToSwitchOn*", _DeviceNumber);
                break;
            case dsSwitchedOn:
                ETHERCAT_INFO("*ELMO %d: dsSwitchedOn*", _DeviceNumber);
                break;
            case dsOperationEnabled:
                ETHERCAT_INFO("*ELMO %d: dsOperationEnabled*", _DeviceNumber);
                break;
            case dsQuickStopActive:
                ETHERCAT_INFO("*ELMO %d: dsQuickStopActive*", _DeviceNumber);
                break;
            case dsFaultReactionActive:
                ETHERCAT_INFO("*ELMO %d: dsFaultReactionActive*", _DeviceNumber);
                break;
            case dsFault:
                ETHERCAT_INFO("*ELMO %d: dsFault*", _DeviceNumber);
                break;
            default:
                ETHERCAT_CRITICAL("*ELMO %d Motor state Error!*", _DeviceNumber);
                break;
        }
        data.motor_last_state = data.drive_state;
    }
}

void ELMO::Reset_Motor_State() noexcept{
    switch(data.drive_state){
            case dsReadyToSwitchOn:
            case dsSwitchedOn:
            case dsOperationEnabled:
                // ETHERCAT_INFO("*Reset ELMO state to dsSwitchOnDisabled*");
                domain_data.RxPDO.ctrl_word = 0x0000;
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, domain_data.RxPDO.ctrl_word);
                break;
            case dsFaultReactionActive:
            case dsFault:
                // ETHERCAT_INFO("*Reset ELMO state to dsSwitchOnDisabled from fault*");
                domain_data.RxPDO.ctrl_word = 0x0080;
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, domain_data.RxPDO.ctrl_word);
                break;
            case dsQuickStopActive:
                break;
            case dsSwitchOnDisabled:
                ETHERCAT_MESSAGE("*Reset ELMO state to dsSwitchOnDisabled complete*");
                data.reset_busy = false;
                data.power_busy = true;
                data.quickstop_busy = false;
                // data.home_busy = false;
                data.position_move_enable = false;
                break;
            case dsNotReadyToSwitchOn:
                // ETHERCAT_WARNING("*Cannot Reset ELMO state from dsNotReadyToSwitchOn*");
                break;
        }
}

void ELMO::Motor_Quickstop() noexcept{
    switch(data.drive_state){
            case dsOperationEnabled:
                // ETHERCAT_WARNING("*Activate motor quickstop*");
                domain_data.RxPDO.ctrl_word = 0x0002;
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, domain_data.RxPDO.ctrl_word);
                break;
            case dsQuickStopActive:
                // ETHERCAT_WARNING("*Motor quickstop activated*");
            case dsSwitchOnDisabled:
                ETHERCAT_MESSAGE("*Motor quickstop complete*");
                data.reset_busy = false;
                data.power_busy = false;
                data.quickstop_busy = false;
                data.home_busy = false;
                data.position_move_enable = false;
                break;
            case dsNotReadyToSwitchOn:
            case dsReadyToSwitchOn:
            case dsSwitchedOn:
            case dsFaultReactionActive:
            case dsFault:
                break;
        }
}

void ELMO::Enable_Motor(const DRIVERMODE _DriveMode) noexcept{
    switch (data.drive_state){
            case dsNotReadyToSwitchOn:
                ETHERCAT_WARNING("*Can not enable ELMO from dsNotReadyToSwitchOn state*");
                break;
            case dsSwitchOnDisabled:
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, 0x0006);
                // ETHERCAT_MESSAGE("*Change ELMO state to dsReadyToSwitchOn*");
                break;
            case dsReadyToSwitchOn:
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, 0x0007);
                // ETHERCAT_MESSAGE("*Change ELMO state to dsSwitchedOn*");
                break;
            case dsSwitchedOn:
                data.drive_mode = _DriveMode;
                // 设置运行模式
                // domain_data.RxPDO.operation_mode = data.drive_mode;
                // EC_WRITE_S8(domain_pd + domain_offset.RxPDO.operation_mode, domain_data.RxPDO.operation_mode);
                EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, 0x000f); // enable operation
                // ETHERCAT_MESSAGE("*Change ELMO state to dsOperationEnabled*");
                domain_data.RxPDO.target_position = domain_data.TxPDO.actual_position; // 将当前位置复制给目标位置，防止使能后电机震动
                if(data.get_origin_point == true)
                {
                    data.move_origin_point = domain_data.TxPDO.actual_position;
                    data.get_origin_point == false;
                }
                EC_WRITE_S32(domain_pd + domain_offset.RxPDO.target_position, domain_data.RxPDO.target_position);
                break;
            case dsOperationEnabled:
                ETHERCAT_MESSAGE("*ELMO: Success to enable*");
                data.reset_busy = false;
                data.power_busy = false;
                data.quickstop_busy = false;
                // data.home_busy = false;
                data.position_move_enable = true;
                break;
            default:
                ETHERCAT_WARNING("*Can not enable ELMO*");
                data.reset_busy = true;
        }
}

void ELMO::Move_Motor_Home() noexcept{
    // 开始回零

    // **************************  启用自定义运动原点 ****************************
    /* if (domain_data.TxPDO.actual_position - data.move_origin_point > 0)
    {
        domain_data.RxPDO.target_position = domain_data.TxPDO.actual_position - (int32_t)HOME_STEP;
        if (domain_data.RxPDO.target_position - data.move_origin_point< 0)
        {
            domain_data.RxPDO.target_position = data.move_origin_point;
        }
    }
    else if (domain_data.TxPDO.actual_position - data.move_origin_point< 0)
        {
            domain_data.RxPDO.target_position = domain_data.TxPDO.actual_position + (int32_t)HOME_STEP;
            if (domain_data.RxPDO.target_position - data.move_origin_point> 0)
            {
                domain_data.RxPDO.target_position = data.move_origin_point;
            }
        }
    else if (domain_data.TxPDO.actual_position == data.move_origin_point)
        {
            domain_data.RxPDO.target_position = data.move_origin_point;
            data.reset_busy = true;
            data.power_busy = false;
            data.quickstop_busy = false;
            data.home_busy = false; // 回零结束
            data.position_move_enable = false;
        }
    EC_WRITE_S32(domain_pd + domain_offset.RxPDO.target_position, domain_data.RxPDO.target_position); */
    // **************************  启用自定义运动原点 ****************************


    // ****************************  启用绝对值原点 *****************************
    if (domain_data.TxPDO.actual_position > 0)
    {
        // domain_data.RxPDO.target_position = domain_data.TxPDO.actual_position - (int32_t)HOME_STEP;
        domain_data.RxPDO.target_position -= (int32_t)HOME_STEP;
        if (domain_data.RxPDO.target_position < 0)
        {
            domain_data.RxPDO.target_position = 0;
            // data.reset_busy = true;
            // data.power_busy = false;
            // data.quickstop_busy = false;
            data.home_busy = false; // 回零结束
            // data.position_move_enable = false;
        }
    }
    else if (domain_data.TxPDO.actual_position < 0)
    {
        // domain_data.RxPDO.target_position = domain_data.TxPDO.actual_position + (int32_t)HOME_STEP;
        domain_data.RxPDO.target_position += (int32_t)HOME_STEP;
        if (domain_data.RxPDO.target_position > 0)
        {
            domain_data.RxPDO.target_position = 0;
            // data.reset_busy = true;
            // data.power_busy = false;
            // data.quickstop_busy = false;
            data.home_busy = false; // 回零结束
            // data.position_move_enable = false;
        }
    }
    else if (domain_data.TxPDO.actual_position == 0)
    {
        domain_data.RxPDO.target_position = 0;
        // data.reset_busy = true;
        // data.power_busy = false;
        // data.quickstop_busy = false;
        data.home_busy = false; // 回零结束
        // data.position_move_enable = false;
    }
    EC_WRITE_S32(domain_pd + domain_offset.RxPDO.target_position, domain_data.RxPDO.target_position);
     // ****************************  启用绝对值原点 *****************************
}

void ELMO::Slave_Exit() noexcept{
    Reset_Motor_State();
}

void ELMO::Read_Data() noexcept{
    switch(TxPDO)
    {
        case 0x1A07:
            domain_data.TxPDO.status_word =
                EC_READ_U16(domain_pd + domain_offset.TxPDO.status_word);
            domain_data.TxPDO.actual_position =
                EC_READ_S32(domain_pd + domain_offset.TxPDO.actual_position);
            // domain_data.TxPDO.di_status =
            //     EC_READ_U32(domain_pd + domain_offset.TxPDO.di_status);
            domain_data.TxPDO.torque_input =
                EC_READ_S16(domain_pd + domain_offset.TxPDO.torque_input);
            // domain_data.TxPDO.mode_display =
            //     EC_READ_S8(domain_pd + domain_offset.TxPDO.mode_display);
            break;
         case 0x1A00:
            domain_data.TxPDO.status_word =
                EC_READ_U16(domain_pd + domain_offset.TxPDO.status_word);
            domain_data.TxPDO.actual_position =
                EC_READ_S32(domain_pd + domain_offset.TxPDO.actual_position);
            domain_data.TxPDO.di_status =
                EC_READ_U32(domain_pd + domain_offset.TxPDO.di_status);
            // domain_data.TxPDO.torque_input =
            //     EC_READ_S16(domain_pd + domain_offset.TxPDO.torque_input);
            // domain_data.TxPDO.mode_display =
            //     EC_READ_S8(domain_pd + domain_offset.TxPDO.mode_display);
            break;
        default:
            ETHERCAT_WARNING("Cannot Read Data from domain because of no TxPDO is configured!");
            break;
    }

    // CIA402 CANOpen over EtherCAT status machine
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

void ELMO::Write_Data() noexcept{
    switch(RxPDO)
    {
        case 0x1607:
            EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, domain_data.RxPDO.ctrl_word);
            EC_WRITE_S32(domain_pd + domain_offset.RxPDO.target_position, domain_data.RxPDO.target_position);
            // EC_WRITE_U32(domain_pd + domain_offset.RxPDO.digital_output, domain_data.RxPDO.digital_output);
            // EC_WRITE_S8(domain_pd + domain_offset.RxPDO.operation_mode, domain_data.RxPDO.operation_mode);
            break;
        case 0x1600:
            EC_WRITE_U16(domain_pd + domain_offset.RxPDO.ctrl_word, domain_data.RxPDO.ctrl_word);
            EC_WRITE_S32(domain_pd + domain_offset.RxPDO.target_position, domain_data.RxPDO.target_position);
            EC_WRITE_U32(domain_pd + domain_offset.RxPDO.digital_output, domain_data.RxPDO.digital_output);
            // EC_WRITE_S8(domain_pd + domain_offset.RxPDO.operation_mode, domain_data.RxPDO.operation_mode);
            break;
        default:
            ETHERCAT_WARNING("Cannot Write Data to domain because of no RxPDO is configured!");
            break;
    }
}

void ELMO::Get_PDO_Entry_Reg() noexcept{
    int count = -1;
    switch (RxPDO)
    {
        case 0x1607:
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, CTRL_WORD, &domain_offset.RxPDO.ctrl_word, NULL};
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, TARGET_POSITION, &domain_offset.RxPDO.target_position, NULL};
            // ELMO_domain_regs[++count] =
            // (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, DO_STATUS, &domain_offset.RxPDO.digital_output, NULL};
            // ELMO_domain_regs[++count] =
            // (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, OPERATION_MODE, &domain_offset.RxPDO.operation_mode, NULL};
            break;
        case 0x1600:
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, CTRL_WORD, &domain_offset.RxPDO.ctrl_word, NULL};
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, TARGET_POSITION, &domain_offset.RxPDO.target_position, NULL};
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, DO_STATUS, &domain_offset.RxPDO.digital_output, NULL};
            // ELMO_domain_regs[++count] =
            // (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, OPERATION_MODE, &domain_offset.RxPDO.operation_mode, NULL};
            break;
        default:
            ETHERCAT_CRITICAL("Cannot config PDO Entry Regs for RxPDO!");
            break;
    }
    switch (TxPDO)
    {
        case 0x1A07:
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, STATUS_WORD, &domain_offset.TxPDO.status_word, NULL};
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, ACTUAL_POSITION, &domain_offset.TxPDO.actual_position, NULL};
            // ELMO_domain_regs[++count] =
            // (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, DI_STATUS, &domain_offset.TxPDO.di_status, NULL};
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, ANALOG_INPUT, &domain_offset.TxPDO.torque_input, NULL};
            // ELMO_domain_regs[++count] =
            // (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, MODE_DISPLAY, &domain_offset.TxPDO.mode_display, NULL};
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){};
            break;
        case 0x1A00:
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, STATUS_WORD, &domain_offset.TxPDO.status_word, NULL};
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, ACTUAL_POSITION, &domain_offset.TxPDO.actual_position, NULL};
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, DI_STATUS, &domain_offset.TxPDO.di_status, NULL};
            // ELMO_domain_regs[++count] =
            // (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, ANALOG_INPUT, &domain_offset.TxPDO.torque_input, NULL};
            // ELMO_domain_regs[++count] =
            // (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, MODE_DISPLAY, &domain_offset.TxPDO.mode_display, NULL};
            ELMO_domain_regs[++count] =
            (ec_pdo_entry_reg_t){};
            break;
        default:
            ETHERCAT_CRITICAL("Cannot config PDO Entry Regs for TxPDO!");
            break;
    }
    domain_regs = ELMO_domain_regs;
}

void ELMO::Get_Sync_Info() noexcept{
    switch(RxPDO)
    {
        case 0x1607:
            ELMO_sync2_pdos[0] = ELMO_pdos[1];
            break;
        case 0x1600:
            ELMO_sync2_pdos[0] = ELMO_pdos[0];
            break;
        default:
            ETHERCAT_ERROR("Cannot config syncs PDO for SM2!");
            break;
    }
    switch(TxPDO)
    {
        case 0x1A07:
            ELMO_sync3_pdos[0] = ELMO_pdos[3];
            break;
        case 0x1A00:
            ELMO_sync3_pdos[0] = ELMO_pdos[2];
            break;
        default:
            ETHERCAT_ERROR("Cannot config syncs PDO for SM3!");
            break;
    }
    ELMO_syncs[0] = (ec_sync_info_t){0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE};
    ELMO_syncs[1] = (ec_sync_info_t){1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE};
    ELMO_syncs[2] = (ec_sync_info_t){2, EC_DIR_OUTPUT, 1, ELMO_sync2_pdos, EC_WD_ENABLE};
    ELMO_syncs[3] = (ec_sync_info_t){3, EC_DIR_INPUT, 1, ELMO_sync3_pdos, EC_WD_DISABLE};
    ELMO_syncs[4] = (ec_sync_info_t){0xff};
    syncs = ELMO_syncs;
}

void ELMO::Get_SDO_Config_Reg() noexcept{
    int count = -1;
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0x6091, 0x01, (uint8_t*)&sdo_data.gear_ratio_num_0x6091_0x01, 4, &data.abort_code};
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0x6091, 0x02, (uint8_t*)&sdo_data.gear_ratio_den_0x6091_0x02, 4, &data.abort_code};
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0x6092, 0x01, (uint8_t*)&sdo_data.feed_constant_num_0x6092_0x01, 4, &data.abort_code};
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0x6092, 0x02, (uint8_t*)&sdo_data.feed_constant_den_0x6092_0x02, 4, &data.abort_code};
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0x608F, 0x01, (uint8_t*)&sdo_data.position_encoder_resolution_num_0x608F_0x01, 4, &data.abort_code};
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0x608F, 0x02, (uint8_t*)&sdo_data.position_encoder_resolution_den_0x608F_0x02, 4, &data.abort_code};
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0x6096, 0x01, (uint8_t*)&sdo_data.velocity_factor_num_0x6096_0x01, 4, &data.abort_code};
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0x6096, 0x02, (uint8_t*)&sdo_data.velocity_factor_den_0x6096_0x02, 4, &data.abort_code};
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0x606A, 0x00, (uint8_t*)&sdo_data.sensor_selection_code_0x606A_0x00, 2, &data.abort_code};
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0x6060, 0x00, (uint8_t*)&sdo_data.operation_mode_0x6060_0x00, 1, &data.abort_code};
    ELMO_sdo_config_regs[++count] =
    (ec_sdo_config_reg_t){0};

    sdo_config_regs = ELMO_sdo_config_regs;
}

void ELMO::Add_Slave_To_Master(EtherCAT_Master& _Master) noexcept{
    _Master.Slaves.push_back((EtherCAT_Slave_Base *)this);
}