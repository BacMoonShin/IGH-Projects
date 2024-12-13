#include "EtherCAT_Master.h"
#include "EL3102.h"

ec_pdo_entry_reg_t EL3102_domain_regs[5];
ec_pdo_entry_info_t EL3102_pdo_entries[] = {/*TxPdo 0x1A00*/
                                            {0x3101, 0x01, 8}, /* Input1 */
                                            {0x3101, 0x02, 16}, /* Input2 */
                                            /*TxPdo 0x1A01*/
                                            {0x3102, 0x01, 8}, /* Input3 */
                                            {0x3102, 0x02, 16}, /* Input4 */
                                           };

ec_pdo_info_t EL3102_pdos[] = {// TxPdo
                                {0x1A00, 2, EL3102_pdo_entries + 0},  /* Channel 1 */
                                {0x1A01, 2, EL3102_pdo_entries + 2},  /* Channel 2 */
                              };

ec_sync_info_t EL3102_syncs[] = {
                          {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                          {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                          {2, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                          {3, EC_DIR_INPUT, 2, EL3102_pdos, EC_WD_DISABLE},
                          {0xff}
                        };

void Init_EL3102(struct EtherCAT_Master *Master, struct EL3102 *Device, uint16_t Alias, uint16_t Position, uint32_t Vendor_id, uint32_t Product_code){

    Device->alias = Alias;
    Device->position = Position;
    Device->vendor_id = Vendor_id;
    Device->product_code = Product_code;
    Device->domain_regs = Get_PDO_Entry_Reg_EL3102(Device);
    Device->syncs = Get_Sync_Info_EL3102(Device);

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
        printf("Failed to get slave configuration for EL3102!\n");
        exit(EXIT_FAILURE); // 配置失败，退出线程
    }

    // 对从站进行配置PDOs
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(Device->slave_config, EC_END, Device->syncs))
    {
        printf("Failed to configure EL3102 PDOs!\n");
        exit(EXIT_FAILURE); // 配置失败，退出线程
    }
    else
    {
        printf("*Success to configuring EL3102 PDOs*\n"); // 配置成功
    }

    // 注册PDO entry
    if (ecrt_domain_reg_pdo_entry_list(Device->domain, Device->domain_regs))
    {
        printf("PDO entry registration failed!\n");
        exit(EXIT_FAILURE); // 注册失败，退出线程
    }
    else
    {
        printf("*Success to configuring EL3102 PDO entry*\n"); // 注册成功
    }
}

void Get_PD_EL3102(struct EL3102 *Device){
    if (!(Device->domain_pd = ecrt_domain_data(Device->domain))){
      printf("%p\n", Device->domain_pd);
      printf("Get domain_pd failed!\n");
      exit(EXIT_FAILURE);
    }
}

void Read_Data_EL3102(struct EL3102 *Device){
    Device->data.channal1_status =
        EC_READ_U16(Device->domain_pd + Device->domain_offset.channal1_status_offset);
    Device->data.channal1_value =
        EC_READ_S32(Device->domain_pd + Device->domain_offset.channal1_value_offset);
    Device->data.channal2_status =
        EC_READ_U16(Device->domain_pd + Device->domain_offset.channal2_status_offset);
    Device->data.channal2_value =
        EC_READ_S32(Device->domain_pd + Device->domain_offset.channal2_value_offset);
}

void Write_Data_EL3102(struct EL3102 *Device){}

ec_pdo_entry_reg_t *Get_PDO_Entry_Reg_EL3102(struct EL3102 *Device){
  uint16_t alias = Device->alias;        // 定义EL3102输出模块在EtherCAT总线上的位置，总站位置
  uint16_t position = Device->position;     // 定义EL3102输出模块在EtherCAT总线上的位置，从站位置
  uint32_t vendor_id = Device->vendor_id;    // 定义EL3102输出模块的厂家标识
  uint32_t product_code = Device->product_code; // 定义EL3102输出模块的产品标识

  EL3102_domain_regs[0] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x3101, 0x01, &Device->domain_offset.channal1_status_offset, NULL};
  EL3102_domain_regs[1] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x3101, 0x02, &Device->domain_offset.channal1_value_offset, NULL};
  EL3102_domain_regs[2] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x3102, 0x01, &Device->domain_offset.channal2_status_offset, NULL};
  EL3102_domain_regs[3] =
  (ec_pdo_entry_reg_t){alias, position, vendor_id, product_code, 0x3102, 0x02, &Device->domain_offset.channal2_value_offset, NULL};
  EL3102_domain_regs[4] =
  (ec_pdo_entry_reg_t){};

  return EL3102_domain_regs;
}

ec_sync_info_t *Get_Sync_Info_EL3102(struct EL3102 *Device){
  return EL3102_syncs;
}