# 从站基类：

- 从站的基础信息：
  - 总站位置
  - 从站位置
  - **TODO：从站工作状态**
  - 从站设备的厂家标识
  - 从站设备的产品标识
  - domain_pd： 过程数据
  - domain *： 域指针
  - domain_state： 域状态
  - ec_slave_config_t *： 从站配置信息位置
  - ec_slave_config_state_t： 从站配置状态
  - ec_pdo_entry_reg_t *： 从站domain_regs指针
  - ec_sync_info_t *： 从站通信配置指针
  - Get_PDO_Entry_Reg()： 获取ec_pdo_entry_reg_t * 从站domain_regs指针函数  ****
  - Get_Sync_Info()： 获取ec_sync_info_t * 从站通信配置指针函数  
  - Add_Slave_To_Master(EtherCAT_Master& Master)： 将从站注册到所属的主站函数
  - Read_Data()： 从站读取数据函数 
  - Write_Data()：从站写入数据函数 



# 从站类：

- 继承从站基类
- 重写纯虚函数
  - **Get_PDO_Entry_Reg()： 获取ec_pdo_entry_reg_t * 从站domain_regs指针函数  TODO：根据从站工作状态构造PDO注册信息**
  - **Get_Sync_Info()： 获取ec_sync_info_t * 从站通信配置指针函数   TODO：根据从站工作状态更改通道选择的PDO列表**
  - **Read_Data()： 从站读取数据函数    TODO：根据从站工作状态选择读取的数据**
  - **Write_Data()：从站写入数据函数    TODO：根据从站工作状态选择写入的数据**
- 特有数据：
  - struct domain_offset： 保存domain访问偏移量的结构体
  - struct data：保存从站设备特有数据的结构体
  - ec_pdo_info_t syncX_pdos[]：保存sync中某一个通道选择的pdo
  - ec_pdo_entry_reg_t domain_regs[]：保存domain注册信息
  - const static ec_pdo_entry_info_t pdo_entries[]：保存从站的所有PDO映射信息
  - const static ec_pdo_info_t pdos[]：保存从站的所有PDO信息
  - const static ec_sync_info_t syncs[]：保存从站以某种模式状态运行时的通信配置信息 
  - Slave_name(EtherCAT_Master& Master, uint16_t Alias, uint16_t Position)：从站构造函数