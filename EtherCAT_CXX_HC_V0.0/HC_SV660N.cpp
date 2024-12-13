#include <unistd.h>
#include <vector>
#include "HC_SV660N.h"

ec_pdo_entry_info_t HC_SV660N::HC_SV660N_pdo_entries[16] = {/*RxPDO 0x1702*/
                                                            {CTRL_WORD, 0x00, 16},
                                                            {TARGET_POSITION, 0x00, 32},
                                                            {TARGET_VELOCITY, 0x00, 32},
                                                            {TARGET_TORQUE, 0x00, 16},
                                                            {OPERATION_MODE, 0x00, 8},
                                                            {SERVO_PROBE, 0x00, 16},
                                                            {MAX_SPEED, 0x00, 32},
                                                            /* TxPDO 0x1B01 */
                                                            {ERROR_CODE, 0x00, 16},
                                                            {STATUS_WORD, 0x00, 16},
                                                            {CURRENT_POSITION, 0x00, 32},
                                                            {CURRENT_TORQUE, 0x00, 16},
                                                            {POSITION_DEVIATION, 0x00, 32},
                                                            {PROBE_STATUS, 0x00, 16},
                                                            {PROBE1_RISE_EDGE_POSITION, 0x00, 32},
                                                            {PROBE2_RISE_EDGE_POSITION, 0x00, 32},
                                                            {DI_STATUS, 0x00, 32},
                                                          };
ec_pdo_info_t HC_SV660N::HC_SV660N_pdos[2] = {// RxPDO
                                              {0x1702, 7, HC_SV660N_pdo_entries + 0},
                                               // TxPDO
                                              {0x1b01, 9, HC_SV660N_pdo_entries + 7},
                                              };


HC_SV660N::HC_SV660N(EtherCAT_Master& Master, uint16_t Alias, uint16_t Position, const struct HC_SV660N_Motor Motor){
    alias = Alias;
    position = Position;
    motor = Motor;
    vendor_id = HC_SV660N_VENDOR_ID;
    product_code = HC_SV660N_PRODUCT_CODE;

    Get_PDO_Entry_Reg();
    Get_Sync_Info();
    Add_Slave_To_Master(Master);
}

void HC_SV660N::Get_PDO_Entry_Reg(){
    HC_SV660N_domain_regs[0] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, CTRL_WORD, 0x00, &domain_offset.ctrl_word, NULL};
    HC_SV660N_domain_regs[1] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, TARGET_POSITION, 0x00, &domain_offset.target_position, NULL};
    HC_SV660N_domain_regs[2] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, TARGET_VELOCITY, 0x00, &domain_offset.target_velocity, NULL};
    HC_SV660N_domain_regs[3] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, TARGET_TORQUE, 0x00, &domain_offset.target_torque, NULL};
    HC_SV660N_domain_regs[4] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, OPERATION_MODE, 0x00, &domain_offset.operation_mode, NULL};
    HC_SV660N_domain_regs[5] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, SERVO_PROBE, 0x00, &domain_offset.servo_probe, NULL};
    HC_SV660N_domain_regs[6] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, MAX_SPEED, 0x00, &domain_offset.max_speed, NULL};
    HC_SV660N_domain_regs[7] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, ERROR_CODE, 0x00, &domain_offset.error_code, NULL};
    HC_SV660N_domain_regs[8] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, STATUS_WORD, 0x00, &domain_offset.status_word, NULL};
    HC_SV660N_domain_regs[9] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, CURRENT_POSITION, 0x00, &domain_offset.current_position, NULL};
    HC_SV660N_domain_regs[10] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, CURRENT_TORQUE, 0x00, &domain_offset.current_torque, NULL};
    HC_SV660N_domain_regs[11] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, POSITION_DEVIATION, 0x00, &domain_offset.position_deviation, NULL};
    HC_SV660N_domain_regs[12] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, PROBE_STATUS, 0x00, &domain_offset.probe_status, NULL};
    HC_SV660N_domain_regs[13] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, PROBE1_RISE_EDGE_POSITION, 0x00, &domain_offset.probe1_rise_edge_position, NULL};
    HC_SV660N_domain_regs[14] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, PROBE2_RISE_EDGE_POSITION, 0x00, &domain_offset.probe2_rise_edge_position, NULL};
    HC_SV660N_domain_regs[15] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, DI_STATUS, 0x00, &domain_offset.di_status, NULL};
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

void HC_SV660N::Add_Slave_To_Master(EtherCAT_Master& Master){
    Master.Slaves.push_back((EtherCAT_Slave_Base *)this);
}

void HC_SV660N::Read_Data(){
    domain_data.error_code =
        EC_READ_U16(domain_pd + domain_offset.error_code);
    domain_data.status_word =
        EC_READ_U16(domain_pd + domain_offset.status_word);
    domain_data.current_position =
        EC_READ_S32(domain_pd + domain_offset.current_position);
    domain_data.current_torque =
        EC_READ_S16(domain_pd + domain_offset.current_torque);
    domain_data.position_deviation =
        EC_READ_S32(domain_pd + domain_offset.position_deviation);
    domain_data.probe_status =
        EC_READ_U16(domain_pd + domain_offset.probe_status);
    domain_data.probe1_rise_edge_position =
        EC_READ_S32(domain_pd + domain_offset.probe1_rise_edge_position);
    domain_data.probe2_rise_edge_position =
        EC_READ_S32(domain_pd + domain_offset.probe2_rise_edge_position);
    domain_data.di_status =
        EC_READ_U32(domain_pd + domain_offset.di_status);

    data.actual_torque = (domain_data.current_torque / 1000) * motor.rated_torque;
}

void HC_SV660N::Write_Data(){
    EC_WRITE_U16(domain_pd + domain_offset.ctrl_word, domain_data.ctrl_word);
    EC_WRITE_S32(domain_pd + domain_offset.target_position, domain_data.target_position);
    EC_WRITE_S32(domain_pd + domain_offset.target_velocity, domain_data.target_velocity);
    EC_WRITE_S16(domain_pd + domain_offset.target_torque, domain_data.target_torque);
    EC_WRITE_U8(domain_pd + domain_offset.operation_mode, domain_data.operation_mode);
    EC_WRITE_U16(domain_pd + domain_offset.servo_probe, domain_data.servo_probe);
    EC_WRITE_U32(domain_pd + domain_offset.max_speed, domain_data.max_speed);
}