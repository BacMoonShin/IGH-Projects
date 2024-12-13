#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include <vector>
#include "EL2889.h"

ec_pdo_entry_info_t EL2889::EL2889_pdo_entries[16] = {/*RxPdo 0x1600*/
                                                      {0x7000, 0x01, 1}, /* Output1 */
                                                      {0x7010, 0x01, 1}, /* Output2 */
                                                      {0x7020, 0x01, 1}, /* Output3 */
                                                      {0x7030, 0x01, 1}, /* Output4 */
                                                      {0x7040, 0x01, 1}, /* Output5 */
                                                      {0x7050, 0x01, 1}, /* Output6 */
                                                      {0x7060, 0x01, 1}, /* Output7 */
                                                      {0x7070, 0x01, 1}, /* Output8 */
                                                      {0x7080, 0x01, 1}, /* Output9 */
                                                      {0x7090, 0x01, 1}, /* Output10 */
                                                      {0x70a0, 0x01, 1}, /* Output11 */
                                                      {0x70b0, 0x01, 1}, /* Output12 */
                                                      {0x70c0, 0x01, 1}, /* Output13 */
                                                      {0x70d0, 0x01, 1}, /* Output14 */
                                                      {0x70e0, 0x01, 1}, /* Output15 */
                                                      {0x70f0, 0x01, 1}, /* Output16 */
                                                     };
ec_pdo_info_t EL2889::EL2889_pdos[16] = {// RxPdo
                                        {0x1600, 1, EL2889_pdo_entries + 0},  /* Channel 1 */
                                        {0x1601, 1, EL2889_pdo_entries + 1},  /* Channel 2 */
                                        {0x1602, 1, EL2889_pdo_entries + 2},  /* Channel 3 */
                                        {0x1603, 1, EL2889_pdo_entries + 3},  /* Channel 4 */
                                        {0x1604, 1, EL2889_pdo_entries + 4},  /* Channel 5 */
                                        {0x1605, 1, EL2889_pdo_entries + 5},  /* Channel 6 */
                                        {0x1606, 1, EL2889_pdo_entries + 6},  /* Channel 7 */
                                        {0x1607, 1, EL2889_pdo_entries + 7},  /* Channel 8 */
                                        {0x1608, 1, EL2889_pdo_entries + 8},  /* Channel 9 */
                                        {0x1609, 1, EL2889_pdo_entries + 9},  /* Channel 10 */
                                        {0x160a, 1, EL2889_pdo_entries + 10}, /* Channel 11 */
                                        {0x160b, 1, EL2889_pdo_entries + 11}, /* Channel 12 */
                                        {0x160c, 1, EL2889_pdo_entries + 12}, /* Channel 13 */
                                        {0x160d, 1, EL2889_pdo_entries + 13}, /* Channel 14 */
                                        {0x160e, 1, EL2889_pdo_entries + 14}, /* Channel 15 */
                                        {0x160f, 1, EL2889_pdo_entries + 15},/* Channel 16 */
                                      };


EL2889::EL2889(EtherCAT_Master& Master, uint16_t Alias, uint16_t Position){
    alias = Alias;
    position = Position;
    vendor_id = 0x00000002;
    product_id = 0x0b493052;

    Get_PDO_Entry_Reg();
    Get_Sync_Info();
    Add_Slave_To_Master(Master);
}

void EL2889::Get_PDO_Entry_Reg(){
    EL2889_domain_regs[0] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x7000, 0x01, &domain_offset.channel1, &domain_offset.channel1_offset};
    EL2889_domain_regs[1] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x7010, 0x01, &domain_offset.channel2, &domain_offset.channel2_offset};
    EL2889_domain_regs[2] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x7020, 0x01, &domain_offset.channel3, &domain_offset.channel3_offset};
    EL2889_domain_regs[3] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x7030, 0x01, &domain_offset.channel4, &domain_offset.channel4_offset};
    EL2889_domain_regs[4] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x7040, 0x01, &domain_offset.channel5, &domain_offset.channel5_offset};
    EL2889_domain_regs[5] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x7050, 0x01, &domain_offset.channel6, &domain_offset.channel6_offset};
    EL2889_domain_regs[6] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x7060, 0x01, &domain_offset.channel7, &domain_offset.channel7_offset};
    EL2889_domain_regs[7] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x7070, 0x01, &domain_offset.channel8, &domain_offset.channel8_offset};
    EL2889_domain_regs[8] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x7080, 0x01, &domain_offset.channel9, &domain_offset.channel9_offset};
    EL2889_domain_regs[9] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x7090, 0x01, &domain_offset.channel10, &domain_offset.channel10_offset};
    EL2889_domain_regs[10] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x70a0, 0x01, &domain_offset.channel11, &domain_offset.channel11_offset};
    EL2889_domain_regs[11] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x70b0, 0x01, &domain_offset.channel12, &domain_offset.channel12_offset};
    EL2889_domain_regs[12] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x70c0, 0x01, &domain_offset.channel13, &domain_offset.channel13_offset};
    EL2889_domain_regs[13] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x70d0, 0x01, &domain_offset.channel14, &domain_offset.channel14_offset};
    EL2889_domain_regs[14] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x70e0, 0x01, &domain_offset.channel15, &domain_offset.channel15_offset};
    EL2889_domain_regs[15] =
    (ec_pdo_entry_reg_t){alias, position, vendor_id, product_id, 0x70f0, 0x01, &domain_offset.channel16, &domain_offset.channel16_offset};
    EL2889_domain_regs[16] =
    (ec_pdo_entry_reg_t){};

    domain_regs = EL2889_domain_regs;
}

void EL2889::Get_Sync_Info(){
    for(int i=0; i<8; i++)
    {
        EL2889_sync0_pdos[i] = EL2889_pdos[i];
        EL2889_sync1_pdos[i] = EL2889_pdos[i+8];
    }
    EL2889_syncs[0] = (ec_sync_info_t){0, EC_DIR_OUTPUT, 8, EL2889_sync0_pdos, EC_WD_ENABLE};
    EL2889_syncs[1] = (ec_sync_info_t){1, EC_DIR_OUTPUT, 8, EL2889_sync1_pdos, EC_WD_ENABLE};
    EL2889_syncs[2] = (ec_sync_info_t){0xff};
    syncs = EL2889_syncs;
}

void EL2889::Add_Slave_To_Master(EtherCAT_Master& Master){
    Master.Slaves.push_back((EtherCAT_Slave_Base *)this);
}

void EL2889::Read_Data(){
    data.channel1 =
        EC_READ_BIT(domain_pd +domain_offset.channel1, domain_offset.channel1_offset);
    data.channel2 =
        EC_READ_BIT(domain_pd +domain_offset.channel2, domain_offset.channel2_offset);
    data.channel3 =
        EC_READ_BIT(domain_pd +domain_offset.channel3, domain_offset.channel3_offset);
    data.channel4 =
        EC_READ_BIT(domain_pd +domain_offset.channel4, domain_offset.channel4_offset);
    data.channel5 =
        EC_READ_BIT(domain_pd +domain_offset.channel5, domain_offset.channel5_offset);
    data.channel6 =
        EC_READ_BIT(domain_pd +domain_offset.channel6, domain_offset.channel6_offset);
    data.channel7 =
        EC_READ_BIT(domain_pd +domain_offset.channel7, domain_offset.channel7_offset);
    data.channel8 =
        EC_READ_BIT(domain_pd +domain_offset.channel8, domain_offset.channel8_offset);
    data.channel9 =
        EC_READ_BIT(domain_pd +domain_offset.channel9, domain_offset.channel9_offset);
    data.channel10 =
        EC_READ_BIT(domain_pd +domain_offset.channel10, domain_offset.channel10_offset);
    data.channel11 =
        EC_READ_BIT(domain_pd +domain_offset.channel11, domain_offset.channel11_offset);
    data.channel12 =
        EC_READ_BIT(domain_pd +domain_offset.channel12, domain_offset.channel12_offset);
    data.channel13 =
        EC_READ_BIT(domain_pd +domain_offset.channel13, domain_offset.channel13_offset);
    data.channel14 =
        EC_READ_BIT(domain_pd +domain_offset.channel14, domain_offset.channel14_offset);
    data.channel15 =
        EC_READ_BIT(domain_pd +domain_offset.channel15, domain_offset.channel15_offset);
    data.channel16 =
        EC_READ_BIT(domain_pd +domain_offset.channel16, domain_offset.channel16_offset);
}

void EL2889::Write_Data(){
    EC_WRITE_BIT(domain_pd + domain_offset.channel1, domain_offset.channel1_offset, data.channel1);
    EC_WRITE_BIT(domain_pd + domain_offset.channel2, domain_offset.channel2_offset, data.channel2);
    EC_WRITE_BIT(domain_pd + domain_offset.channel3, domain_offset.channel3_offset, data.channel3);
    EC_WRITE_BIT(domain_pd + domain_offset.channel4, domain_offset.channel4_offset, data.channel4);
    EC_WRITE_BIT(domain_pd + domain_offset.channel5, domain_offset.channel5_offset, data.channel5);
    EC_WRITE_BIT(domain_pd + domain_offset.channel6, domain_offset.channel6_offset, data.channel6);
    EC_WRITE_BIT(domain_pd + domain_offset.channel7, domain_offset.channel7_offset, data.channel7);
    EC_WRITE_BIT(domain_pd + domain_offset.channel8, domain_offset.channel8_offset, data.channel8);
    EC_WRITE_BIT(domain_pd + domain_offset.channel9, domain_offset.channel9_offset, data.channel9);
    EC_WRITE_BIT(domain_pd + domain_offset.channel10, domain_offset.channel10_offset, data.channel10);
    EC_WRITE_BIT(domain_pd + domain_offset.channel11, domain_offset.channel11_offset, data.channel11);
    EC_WRITE_BIT(domain_pd + domain_offset.channel12, domain_offset.channel12_offset, data.channel12);
    EC_WRITE_BIT(domain_pd + domain_offset.channel13, domain_offset.channel13_offset, data.channel13);
    EC_WRITE_BIT(domain_pd + domain_offset.channel14, domain_offset.channel14_offset, data.channel14);
    EC_WRITE_BIT(domain_pd + domain_offset.channel15, domain_offset.channel15_offset, data.channel15);
    EC_WRITE_BIT(domain_pd + domain_offset.channel16, domain_offset.channel16_offset, data.channel16);
}