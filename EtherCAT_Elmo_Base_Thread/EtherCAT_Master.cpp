#include "EtherCAT_Master.h"
#include "EtherCAT_Slave.h"

uint8_t EtherCAT_Master::_Master_Number_ = 0;

EtherCAT_Master::EtherCAT_Master(){}
EtherCAT_Master::~EtherCAT_Master(){}

void EtherCAT_Master::EtherCAT_Master_Config(){
    EtherCAT_Master_Init();

    for(int i = 0; i < Slaves.size(); ++i){
        EtherCAT_Master_Config_Slave(Slaves[i], i);
#ifdef ENABLE_DC
        EtherCAT_Master_Config_Slave_DC(Slaves[i], i);
#endif
#ifdef ENABLE_SDO
        EtherCAT_Master_Config_Slave_SDO(Slaves[i], i);
#endif
    }

#ifdef ENABLE_DC
        ecrt_master_select_reference_clock(master, NULL);
#endif

    EtherCAT_Master_Activate();

    for(int i = 0; i < Slaves.size(); ++i){
        EtherCAT_Get_PD_For_Slave(Slaves[i], i);
    }
}

void EtherCAT_Master::EtherCAT_Check_States(){
    // 检查过程数据状态（可选）
    for(int i = 0; i < Slaves.size(); ++i){
        Check_Domain_State(Slaves[i]->domain, &Slaves[i]->domain_state, i);
    }
    // 检查主站状态
    Check_Master_State(master, &master_state);
    // 检查从站配置状态
    for(int i = 0; i < Slaves.size(); ++i){
        Check_Slave_Config_States(Slaves[i]->slave_config, &Slaves[i]->slave_config_state, i);
    }
}

void EtherCAT_Master::EtherCAT_Master_Receive(){
    ecrt_master_receive(master);                 // EtherCAT 主站接收一次报文
    for(int i = 0; i < Slaves.size(); ++i){
        ecrt_domain_process(Slaves[i]->domain);  // 可以理解成根据报文更新主站下每个从站domain域中的信息
        Slaves[i]->Read_Data();                  // 从站读取数据，根据注册获得的PDO对象字典的偏移量进行读取
    }
}

void EtherCAT_Master::EtherCAT_Master_Send(){
    for(int i = 0; i < Slaves.size(); ++i){
        ecrt_domain_queue(Slaves[i]->domain);    // 根据domain数据生成帧
    }
    ecrt_master_send(master);                    // EtherCAT 主站发送报文
}

void EtherCAT_Master::EtherCAT_Master_Exit(){
    ETHERCAT_INFO("*Stopping EtherCAT Master %d...*", master_index);
    EtherCAT_Master_Receive();
    for(int i = 0; i < Slaves.size(); ++i){
        Slaves[i]->Slave_Exit();
    }
    EtherCAT_Master_Send();  // EtherCAT主站发送
    EtherCAT_Master_Deactivate();
    ETHERCAT_INFO("*EtherCAT Master %d stopped*", master_index);
    ecrt_release_master(master);
}

void EtherCAT_Master::EtherCAT_Master_Init(){
    // 创建ethercat主站master
    master = ecrt_request_master(_Master_Number_);
    if (!master)
    {
        ETHERCAT_ERROR("*Failed to create EtherCAT Master %d*", _Master_Number_);
    }
    else{
        ETHERCAT_MESSAGE("*Success to create EtherCAT Master %d*", _Master_Number_);
    }
    master_index = _Master_Number_;
    ++_Master_Number_;
}

void EtherCAT_Master::EtherCAT_Master_Config_Slave(EtherCAT_Slave_Base * _Slave, int _Slave_Index){
    // 创建域domain
    _Slave->domain = ecrt_master_create_domain(master);
    if (!_Slave->domain)
    {
        ETHERCAT_ERROR("*Failed to create master domain for slave %d!*", _Slave_Index);
    }
    else
    {
        ETHERCAT_MESSAGE("*Success to create master domain for slave %d*", _Slave_Index);
    }

    // 配置从站
    if (!(_Slave->slave_config = ecrt_master_slave_config(master, _Slave->alias, _Slave->position, _Slave->vendor_id, _Slave->product_code)))
    {
        ETHERCAT_ERROR("*Failed to get slave configuration for slave %d!*", _Slave_Index);
    }
    else
    {
        ETHERCAT_MESSAGE("*Success to get slave configuration for slave %d*", _Slave_Index);
    }

    // 对从站进行配置PDOs
    // ETHERCAT_INFO("Configuring PDOs...");
    if (ecrt_slave_config_pdos(_Slave->slave_config, EC_END, _Slave->syncs))
    {
        ETHERCAT_ERROR("*Failed to configure PDOs for slave %d!*", _Slave_Index);
    }
    else
    {
        ETHERCAT_MESSAGE("*Success to configuring PDOs for slave %d*", _Slave_Index);
    }

    // 注册PDO entry
    if (ecrt_domain_reg_pdo_entry_list(_Slave->domain, _Slave->domain_regs))
    {
        ETHERCAT_ERROR("*PDO entry registration failed for slave %d!*", _Slave_Index);
    }
    else
    {
        ETHERCAT_MESSAGE("*Success to configuring PDO entry for slave %d*", _Slave_Index);
    }
}

void EtherCAT_Master::EtherCAT_Master_Config_Slave_DC(EtherCAT_Slave_Base * _Slave, int _Slave_Index){
    ecrt_slave_config_dc(_Slave->slave_config, _Slave->assign_activate_word, TASK_PERIOD_NS, SHIFT_NS, 0, 0);
    ETHERCAT_MESSAGE("*Success to config DC Clock for Slave %d*", _Slave_Index);
}

void EtherCAT_Master::EtherCAT_Master_Config_Slave_SDO(EtherCAT_Slave_Base * _Slave, int _Slave_Index){
    // 该处也可以使用 ecrt_slave_config_sdo 函数进行配置
    if((*(_Slave->sdo_config_regs)).index != 0){
        const ec_sdo_config_reg_t *sdo_config_reg = _Slave->sdo_config_regs;
        while((*sdo_config_reg).index != 0){
            // if(0 > ecrt_master_sdo_download(master, Slave->position, (*sdo_config_reg).index, (*sdo_config_reg).subindex, (*sdo_config_reg).data, (*sdo_config_reg).data_size, (*sdo_config_reg).abort_code))
            // {
            //     ETHERCAT_ERROR("*Config SDO %04X %04X error! Abort code: %04X*", (*sdo_config_reg).index, (*sdo_config_reg).subindex, *((*sdo_config_reg).abort_code));
            // }
            if(0 > ecrt_slave_config_sdo(_Slave->slave_config, (*sdo_config_reg).index, (*sdo_config_reg).subindex, (*sdo_config_reg).data, (*sdo_config_reg).data_size)
            )
            {
                ETHERCAT_ERROR("*Config SDO %04X %02X error!*", (*sdo_config_reg).index, (*sdo_config_reg).subindex);
            }
            sdo_config_reg++;
        }
        ETHERCAT_MESSAGE("*Success to config SDO for Slave %d*", _Slave_Index);
    }
    else
    {
        ETHERCAT_WARNING("*Slave %d has no SDO to config*", _Slave_Index);
    }
}

void EtherCAT_Master::EtherCAT_Master_Activate(){
    // 激活主站master
    // ETHERCAT_INFO("Activating master...");
    if (ecrt_master_activate(master))
    {
        ETHERCAT_ERROR("*Failed to activate EtherCAT Master %d!*", master_index);
    }
    else
    {
        ETHERCAT_MESSAGE("*EtherCAT Master %d activated*", master_index);
    }
}

void EtherCAT_Master::EtherCAT_Master_Deactivate(){
    ecrt_master_deactivate(master);
}

void EtherCAT_Master::EtherCAT_Get_PD_For_Slave(EtherCAT_Slave_Base * _Slave, int _Slave_Index){
    if (!(_Slave->domain_pd = ecrt_domain_data(_Slave->domain))){
        ETHERCAT_ERROR("*Get domain_pd failed for slave %d!*", _Slave_Index);
    }
    else
    {
        ETHERCAT_MESSAGE("*Success to get domain_pd for slave %d*", _Slave_Index);
    }
}

void EtherCAT_Master::Check_Domain_State(ec_domain_t *_Domain, ec_domain_state_t *_Domain_State, int _Slave_Index)
{
    static ec_domain_state_t domain_state;
    ecrt_domain_state(_Domain, &domain_state);
    if (domain_state.working_counter != _Domain_State->working_counter)
    {
        ETHERCAT_INFO("Slave %d Domain: Working Counter %u.", _Slave_Index, domain_state.working_counter);
    }
    if (domain_state.wc_state != _Domain_State->wc_state)
    {
        switch(domain_state.wc_state){
            case EC_WC_ZERO:
                ETHERCAT_INFO("Slave %d: Domain State: EC_WC_ZERO. No registered process data were exchanged.", _Slave_Index);
                break;
            case EC_WC_INCOMPLETE:
                ETHERCAT_INFO("Slave %d: Domain State: EC_WC_INCOMPLETE. Some of the registered process data were exchanged.", _Slave_Index);
                break;
            case EC_WC_COMPLETE:
                 ETHERCAT_INFO("Slave %d: Domain State: EC_WC_COMPLETE. All registered process data were exchanged.", _Slave_Index);
                break;
        }
        // ETHERCAT_INFO("Slave %d: Domain: State %u.", Slave_Index, domain_state.wc_state);
    }
    *_Domain_State = domain_state;
}

void EtherCAT_Master::Check_Master_State(ec_master_t *_Master, ec_master_state_t *_Master_State)
{
    static ec_master_state_t master_state;
    ecrt_master_state(_Master, &master_state);
    if (master_state.slaves_responding != _Master_State->slaves_responding)
    {
        ETHERCAT_INFO("EtherCAT Master: There is(are) %u slave(s) responding.", master_state.slaves_responding);
    }
    if (master_state.al_states != _Master_State->al_states)
    {
        switch(master_state.al_states){
            case 1:
                ETHERCAT_INFO("EtherCAT Master State: INIT.");
                break;
            case 2:
                ETHERCAT_INFO("EtherCAT Master State: PREOP.");
                break;
            case 4:
                ETHERCAT_INFO("EtherCAT Master State: SAFEOP.");
                break;
            case 8:
                ETHERCAT_INFO("EtherCAT Master State: OP.");
                break;
        }
        // ETHERCAT_INFO("EtherCAT Master: AL states: 0x%02X.", master_state.al_states);
    }
    if (master_state.link_up != _Master_State->link_up)
    {
        ETHERCAT_INFO("EtherCAT Master: Link is %s.", master_state.link_up ? "up" : "down");
    }
    *_Master_State = master_state;
}

void EtherCAT_Master::Check_Slave_Config_States(ec_slave_config_t *_Slave, ec_slave_config_state_t *_Slave_State, int _Slave_Index)
{
    static ec_slave_config_state_t slave_state;
    ecrt_slave_config_state(_Slave, &slave_state);
    if (slave_state.online != _Slave_State->online)
    {
        ETHERCAT_MESSAGE("Slave %d: %s.", _Slave_Index, slave_state.online ? "online" : "offline");
    }

    if (slave_state.operational != _Slave_State->operational)
    {
        if(slave_state.operational){
            ETHERCAT_MESSAGE("Slave %d: operational.", _Slave_Index);
        }
        else{
            ETHERCAT_INFO("Slave %d: Not operational.", _Slave_Index);
        }
    }

    if (slave_state.al_state != _Slave_State->al_state)
    {
        switch(slave_state.al_state){
        case 1:
            // ETHERCAT_INFO("Slave %d: State 0x%02X.", Slave_Index, slave_state.al_state);
            ETHERCAT_INFO("Slave %d State: INIT.", _Slave_Index);
            break;
        case 2:
            ETHERCAT_INFO("Slave %d State: PREOP.", _Slave_Index);
            break;
        case 4:
            ETHERCAT_INFO("Slave %d State: SAFEOP.", _Slave_Index);
            break;
        case 8:
            ETHERCAT_INFO("Slave %d State: OP.", _Slave_Index);
            break;
        }
    }
    *_Slave_State = slave_state;
}
