#include "EtherCAT_Global.h"

#ifndef _ETHERCAT_MASTER_H_
#define _ETHERCAT_MASTER_H_

class EtherCAT_Slave_Base;

class EtherCAT_Master
{
  public:
    static uint8_t _Master_Number_;

    ec_master_t *master;            // 主站
    ec_master_state_t master_state; // 主站状态
    uint8_t master_index;           // 主站索引

    std::vector<EtherCAT_Slave_Base *> Slaves; // 所有从站设备的列表

    EtherCAT_Master();
    ~EtherCAT_Master();
    void EtherCAT_Master_Config();
    void EtherCAT_Check_States();
    void EtherCAT_Master_Receive();
    void EtherCAT_Master_Send();
    void EtherCAT_Master_Exit();

  private:
    void EtherCAT_Master_Init();
    void EtherCAT_Master_Config_Slave(EtherCAT_Slave_Base * _Slave, int _Slave_Index);
    void EtherCAT_Master_Config_Slave_DC(EtherCAT_Slave_Base * _Slave, int _Slave_Index);
    void EtherCAT_Master_Config_Slave_SDO(EtherCAT_Slave_Base * _Slave, int _Slave_Index);
    void EtherCAT_Master_Activate();
    void EtherCAT_Master_Deactivate();
    void EtherCAT_Get_PD_For_Slave(EtherCAT_Slave_Base * _Slave, int _Slave_Index);
    void Check_Domain_State(ec_domain_t *_Domain, ec_domain_state_t *_Domain_State, int _Slave_Index);
    void Check_Master_State(ec_master_t *_Master, ec_master_state_t *_Master_State);
    void Check_Slave_Config_States(ec_slave_config_t *_Slave, ec_slave_config_state_t *_Slave_State, int _Slave_Index);
};


#endif