#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include "EtherCAT_Master.h"
#include "EL2889.h"

ec_pdo_entry_reg_t EL2889_domain_regs[17];
ec_pdo_entry_info_t EL2889_pdo_entries[] = {/*RxPdo 0x1600*/
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

ec_pdo_info_t EL2889_pdos[] = {// RxPdo
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

ec_sync_info_t EL2889_syncs[] = {
                          {0, EC_DIR_OUTPUT, 8, EL2889_pdos + 0, EC_WD_ENABLE},
                          {1, EC_DIR_OUTPUT, 8, EL2889_pdos + 8, EC_WD_ENABLE},
                          {0xff}
                        };

void Init_EL2889(struct EtherCAT_Master *Master, struct EL2889 *Device, uint16_t Alias, uint16_t Position, uint32_t Vendor_id, uint32_t Product_code){
    Device->alias = Alias;
    Device->position = Position;
    Device->vendor_id = Vendor_id;
    Device->product_code = Product_code;
    Device->domain_regs = Get_PDO_Entry_Reg_EL2889(Device);
    Device->syncs = Get_Sync_Info_EL2889(Device);

    // 创建域domain
    Device->domain = ecrt_master_create_domain(Master->master);
    if (!Device->domain)
    {
        printf("Failed to create master domain!\n");
        exit(EXIT_FAILURE); // 创建失败，退出线程
    }

    // 配置从站
    if (!(Device->slave_config = ecrt_master_slave_config(Master->master, Device->alias, Device->position, Device->vendor_id, Device->product_code)))
    {
        printf("Failed to get slave configuration for EL2889!\n");
        exit(EXIT_FAILURE); // 配置失败，退出线程
    }

    // 对从站进行配置PDOs
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(Device->slave_config, EC_END, Device->syncs))
    {
        printf("Failed to configure EL2889 PDOs!\n");
        exit(EXIT_FAILURE); // 配置失败，退出线程
    }
    else
    {
        printf("*Success to configuring EL2889 PDOs*\n"); // 配置成功
    }

    // 注册PDO entry
    if (ecrt_domain_reg_pdo_entry_list(Device->domain, Device->domain_regs))
    {
        printf("PDO entry registration failed!\n");
        exit(EXIT_FAILURE); // 注册失败，退出线程
    }
    else
    {
        printf("*Success to configuring EL2889 PDO entry*\n"); // 注册成功
    }
}

void Get_PD_EL2889(struct EL2889 *Device){
    if (!(Device->domain_pd = ecrt_domain_data(Device->domain))){
      printf("%p\n", Device->domain_pd);
      printf("Get domain_pd failed!\n");
      exit(EXIT_FAILURE);
    }
}

void Read_Data_EL2889(struct EL2889 *Device){
    Device->data.channel1 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel1, Device->domain_offset.channel1_offset);
    Device->data.channel2 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel2, Device->domain_offset.channel2_offset);
    Device->data.channel3 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel3, Device->domain_offset.channel3_offset);
    Device->data.channel4 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel4, Device->domain_offset.channel4_offset);
    Device->data.channel5 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel5, Device->domain_offset.channel5_offset);
    Device->data.channel6 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel6, Device->domain_offset.channel6_offset);
    Device->data.channel7 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel7, Device->domain_offset.channel7_offset);
    Device->data.channel8 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel8, Device->domain_offset.channel8_offset);
    Device->data.channel9 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel9, Device->domain_offset.channel9_offset);
    Device->data.channel10 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel10, Device->domain_offset.channel10_offset);
    Device->data.channel11 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel11, Device->domain_offset.channel11_offset);
    Device->data.channel12 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel12, Device->domain_offset.channel12_offset);
    Device->data.channel13 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel13, Device->domain_offset.channel13_offset);
    Device->data.channel14 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel14, Device->domain_offset.channel14_offset);
    Device->data.channel15 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel15, Device->domain_offset.channel15_offset);
    Device->data.channel16 =
        EC_READ_BIT(Device->domain_pd + Device->domain_offset.channel16, Device->domain_offset.channel16_offset);
}

void Write_Data_EL2889(struct EL2889 *Device){
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel1, Device->domain_offset.channel1_offset, Device->data.channel1);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel2, Device->domain_offset.channel2_offset, Device->data.channel2);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel3, Device->domain_offset.channel3_offset, Device->data.channel3);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel4, Device->domain_offset.channel4_offset, Device->data.channel4);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel5, Device->domain_offset.channel5_offset, Device->data.channel5);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel6, Device->domain_offset.channel6_offset, Device->data.channel6);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel7, Device->domain_offset.channel7_offset, Device->data.channel7);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel8, Device->domain_offset.channel8_offset, Device->data.channel8);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel9, Device->domain_offset.channel9_offset, Device->data.channel9);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel10, Device->domain_offset.channel10_offset, Device->data.channel10);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel11, Device->domain_offset.channel11_offset, Device->data.channel11);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel12, Device->domain_offset.channel12_offset, Device->data.channel12);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel13, Device->domain_offset.channel13_offset, Device->data.channel13);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel14, Device->domain_offset.channel14_offset, Device->data.channel14);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel15, Device->domain_offset.channel15_offset, Device->data.channel15);
    EC_WRITE_BIT(Device->domain_pd + Device->domain_offset.channel16, Device->domain_offset.channel16_offset, Device->data.channel16);
}

ec_pdo_entry_reg_t *Get_PDO_Entry_Reg_EL2889(struct EL2889 *Device){
  uint16_t alias = Device->alias;        // 定义EL2889输出模块在EtherCAT总线上的位置，总站位置
  uint16_t position = Device->position;     // 定义EL2889输出模块在EtherCAT总线上的位置，从站位置
  uint32_t vendor_id = Device->vendor_id;    // 定义EL2889输出模块的厂家标识
  uint32_t product_code = Device->product_code; // 定义EL2889输出模块的产品标识

  EL2889_domain_regs[0] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x7000, 0x01, &Device->domain_offset.channel1, &Device->domain_offset.channel1_offset};
  EL2889_domain_regs[1] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x7010, 0x01, &Device->domain_offset.channel2, &Device->domain_offset.channel2_offset};
  EL2889_domain_regs[2] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x7020, 0x01, &Device->domain_offset.channel3, &Device->domain_offset.channel3_offset};
  EL2889_domain_regs[3] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x7030, 0x01, &Device->domain_offset.channel4, &Device->domain_offset.channel4_offset};
  EL2889_domain_regs[4] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x7040, 0x01, &Device->domain_offset.channel5, &Device->domain_offset.channel5_offset};
  EL2889_domain_regs[5] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x7050, 0x01, &Device->domain_offset.channel6, &Device->domain_offset.channel6_offset};
  EL2889_domain_regs[6] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x7060, 0x01, &Device->domain_offset.channel7, &Device->domain_offset.channel7_offset};
  EL2889_domain_regs[7] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x7070, 0x01, &Device->domain_offset.channel8, &Device->domain_offset.channel8_offset};
  EL2889_domain_regs[8] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x7080, 0x01, &Device->domain_offset.channel9, &Device->domain_offset.channel9_offset};
  EL2889_domain_regs[9] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x7090, 0x01, &Device->domain_offset.channel10, &Device->domain_offset.channel10_offset};
  EL2889_domain_regs[10] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x70a0, 0x01, &Device->domain_offset.channel11, &Device->domain_offset.channel11_offset};
  EL2889_domain_regs[11] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x70b0, 0x01, &Device->domain_offset.channel12, &Device->domain_offset.channel12_offset};
  EL2889_domain_regs[12] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x70c0, 0x01, &Device->domain_offset.channel13, &Device->domain_offset.channel13_offset};
  EL2889_domain_regs[13] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x70d0, 0x01, &Device->domain_offset.channel14, &Device->domain_offset.channel14_offset};
  EL2889_domain_regs[14] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x70e0, 0x01, &Device->domain_offset.channel15, &Device->domain_offset.channel15_offset};
  EL2889_domain_regs[15] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x70f0, 0x01, &Device->domain_offset.channel16, &Device->domain_offset.channel16_offset};
  EL2889_domain_regs[16] =
  (ec_pdo_entry_reg_t){};

  return EL2889_domain_regs;
}

ec_sync_info_t *Get_Sync_Info_EL2889(struct EL2889 *Device){
  return EL2889_syncs;
}