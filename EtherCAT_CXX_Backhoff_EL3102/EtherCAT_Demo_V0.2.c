#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include "EL2889.h"
#include "EL3102.h"
#include "EtherCAT_Master.h"

// ethercat主站应用程序所用的参数
#define TASK_FREQUENCY 1000                                          //Hz 任务周期（任务频率）
#define ENCODER_RESOLUTION 131072                                    // 编码器分辨率
#define HOME_VECOLITY 5                                              // r/s，回零速度
#define HOME_STEP HOME_VECOLITY *ENCODER_RESOLUTION / TASK_FREQUENCY // pulse 回零步长 = 回零速度 * 编码器分辨率 * 任务周期
#define POSITION_STEP 1 / TASK_FREQUENCY                             // 位置模式下步长
#define Pi 3.141592654                                               // 圆周率

// 从站配置所用的参数XQ
#define EL2889_ALIAS 0
#define EL2889_INDEX 2
#define EL2889_VENDOR_ID 0x00000002
#define EL2889_PRODUCT_ID 0x0b493052
#define EL3102_ALIAS 0
#define EL3102_INDEX 1
#define EL3102_VENDOR_ID 0x00000002
#define EL3102_PRODUCT_ID 0x0c1e3052


// CoE对象字典
#define RXPDO 0x1600   // RxPDO的映射表在对象字典中的索引位置
#define TXPDO 0x1A00   // TxPDO的映射表在对象字典中的索引位置

/*CiA 402数据对象(Data Object)*/
#define CTRL_WORD 0x6040        // 控制字的数据对象
#define STATUS_WORD 0x6041      // 状态字的数据对象
#define OPERATION_MODE 0x6060   // 设定运行模式的数据对象
#define MODE_DISPLAY 0x6061     // 当前运行模式的数据对象
#define CURRENT_POSITION 0x6064 // 当前位置的数据对象
#define CURRENT_VELOCITY 0x606C // 当前速度的数据对象
#define TARGET_POSITION 0x607A  // 目标位置的数据对象
#define TARGET_VELOCITY 0x60FF  // 目标速度的数据对象

static void
check_domain_state(ec_domain_t *domain, ec_domain_state_t *domain_state)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain, &ds);
    if (ds.working_counter != domain_state->working_counter)
    {
        printf("Domain: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain_state->wc_state)
    {
        printf("Domain: State %u.\n", ds.wc_state);
    }
    *domain_state = ds;
}

static void
check_master_state(ec_master_t *master, ec_master_state_t *master_state)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state->slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state->al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state->link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    *master_state = ms;
}

static void
check_slave_config_states(ec_slave_config_t *slave,
                          ec_slave_config_state_t *slave_state)
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(slave, &s);
    if (s.al_state != slave_state->al_state)
    {
        printf("EL3102: State 0x%02X.\n", s.al_state);
    }
    if (s.online != slave_state->online)
    {
        printf("EL3102: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != slave_state->operational)
    {
        printf("EL3102: %soperational.\n", s.operational ? "" : "Not ");
    }
    *slave_state = s;
}


int main(int argc, char **argv)
{
    struct EtherCAT_Master Master;
    struct EL3102 el3102;
    EtherCAT_Master_Init(&Master);
    Init_EL3102(&Master, &el3102, EL3102_ALIAS, EL3102_INDEX, EL3102_VENDOR_ID, EL3102_PRODUCT_ID);
    EtherCAT_Master_Activate(&Master);
    Get_PD_EL3102(&el3102);
    printf("*It's working now*\n");

    int stop_print_time = 250;
    bool state = false;
    while (true)
    {
        usleep(1*1000*1000/TASK_FREQUENCY); // 计算一次周期的时间

        // 接收过程数据
        ecrt_master_receive(Master.master); // EtherCAT 主站接收一次报文
        ecrt_domain_process(el3102.domain); // 可以理解成根据报文更新一次domain域中的信息

        // 检查过程数据状态（可选）
        check_domain_state(el3102.domain, &el3102.domain_state);
        // 检查主站状态
        check_master_state(Master.master, &Master.master_state);
        // 检查从站配置状态
        check_slave_config_states(el3102.slave_config, &el3102.slave_config_state);

        if(!(el3102.slave_config_state.operational)){
            stop_print_time = stop_print_time - 1;
            if(stop_print_time == 0){
              printf("*Waiting EL3102 to become operational*\n");
              stop_print_time = 250;
            }
        }
        else{
            // 读取数据，根据注册获得的PDO对象字典的偏移量进行读取
            Read_Data_EL3102(&el3102);

            stop_print_time = stop_print_time - 1;
            if(stop_print_time == 0){
              printf("EL3102:  channel1 status = %2X , channel1 value= %d , channel2 status = %2X , channel2 value = %d\n",
                     el3102.data.channal1_status, el3102.data.channal1_value, el3102.data.channal2_status, el3102.data.channal2_value);
              stop_print_time = 250;
            }
        }
        ecrt_domain_queue(el3102.domain); // 根据domain数据生成帧
        ecrt_master_send(Master.master);  // EtherCAT主站发送
    }
    ecrt_master_deactivate(Master.master);
}