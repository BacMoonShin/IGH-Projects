#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include "EL2889.h"
#include "EtherCAT_Master.h"
#include "EtherCAT_Slave.h"

// ethercat主站应用程序所用的参数
#define TASK_FREQUENCY 1000                                            //*Hz* 任务周期（任务频率）
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

/*
// DS402 CANOpen over EtherCAT 驱动器状态
enum DRIVERSTATE
{
    dsNotReadyToSwitchOn = 0, // 初始化 未完成状态
    dsSwitchOnDisabled,       // 初始化 完成状态
    dsReadyToSwitchOn,        // 主电路电源OFF状态
    dsSwitchedOn,             // 伺服OFF/伺服准备
    dsOperationEnabled,       // 伺服ON
    dsQuickStopActive,        // 即停止
    dsFaultReactionActive,    // 异常（报警）判断
    dsFault                   // 异常（报警）状态
};
*/

/*
// 迈信伺服驱动器里PDO入口的偏移量
// 我们需要定义一些变量去关联需要用到的从站的PD0对象
// PDO对象里每一个控制位都是四个字节，所以在这个结构体中所有的控制位都为unsigned int
struct DRIVERVARIABLE
{
    unsigned int ctrl_word;       // 控制字
    unsigned int operation_mode;  // 设定运行模式
    unsigned int target_velocity; // 目标速度 （pulse/s)
    unsigned int target_postion;  // 目标位置 （pulse）
    unsigned int status_word;      // 状态字
    unsigned int mode_display;     // 实际运行模式
    unsigned int current_velocity; // 当前速度 （pulse/s）
    unsigned int current_postion;  // 当前位置 （pulse）
};
*/


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
        printf("EL2889: State 0x%02X.\n", s.al_state);
    }
    if (s.online != slave_state->online)
    {
        printf("EL2889: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != slave_state->operational)
    {
        printf("EL2889: %soperational.\n", s.operational ? "" : "Not ");
    }
    *slave_state = s;
}


int main(int argc, char **argv)
{
    EtherCAT_Master Master;
    EL2889 el2889(Master, 0, 2);
    Master.EtherCAT_Master_Config();
    printf("*It's working now*\n");

    int stop_change_time = 500;
    int stop_print_time = 250;
    bool state = false;
    while (true)
    {
        usleep(1*1000*1000/TASK_FREQUENCY); // 计算一次周期的时间

        // 接收过程数据
        ecrt_master_receive(Master.master); // EtherCAT 主站接收一次报文
        ecrt_domain_process(el2889.domain); // 可以理解成根据报文更新一次domain域中的信息

        // 检查过程数据状态（可选）
        check_domain_state(el2889.domain, &el2889.domain_state);
        // 检查主站状态
        check_master_state(Master.master, &Master.master_state);
        // 检查从站配置状态
        check_slave_config_states(el2889.slave_config, &el2889.slave_config_state);

        if(!(el2889.slave_config_state.operational)){
            stop_print_time = stop_print_time - 1;
            if(stop_print_time == 0){
              printf("*Waiting EL2889 to become operational*\n");
              stop_print_time = 250;
            }
        }
        else{
            // 读取数据，根据注册获得的PDO对象字典的偏移量进行读取
            // el2889.Read_Data();

            stop_print_time = stop_print_time - 1;
            if(stop_print_time == 0){
              printf("EL2889:  channel0 = %d , channel1 = %d , channel2 = %d , channel3 = %d , channel4 = %d , channel5 = %d , channel6 = %d , channel7 = %d , channel8 = %d , channel9 = %d , channel10 = %d , channel11 = %d , channel12 = %d , channel13 = %d , channel14 = %d , channel15 = %d\n",
                     el2889.data.channel1, el2889.data.channel2, el2889.data.channel3, el2889.data.channel4, el2889.data.channel5, el2889.data.channel6, el2889.data.channel7, el2889.data.channel8, el2889.data.channel9, el2889.data.channel10, el2889.data.channel11, el2889.data.channel12, el2889.data.channel13, el2889.data.channel14, el2889.data.channel15, el2889.data.channel16);
              stop_print_time = 250;
            }

            if(state){
                el2889.data.channel1 = true;
                el2889.data.channel2 = true;
                el2889.data.channel3 = true;
                el2889.data.channel4 = true;
                el2889.data.channel5 = true;
                el2889.data.channel6 = true;
                el2889.data.channel7 = true;
                el2889.data.channel8 = true;
                el2889.data.channel9 = false;
                el2889.data.channel10 = false;
                el2889.data.channel11 = false;
                el2889.data.channel12 = false;
                el2889.data.channel13 = false;
                el2889.data.channel14 = false;
                el2889.data.channel15 = false;
                el2889.data.channel16 = false;
            }
            else{
                el2889.data.channel1 = false;
                el2889.data.channel2 = false;
                el2889.data.channel3 = false;
                el2889.data.channel4 = false;
                el2889.data.channel5 = false;
                el2889.data.channel6 = false;
                el2889.data.channel7 = false;
                el2889.data.channel8 = false;
                el2889.data.channel9 = true;
                el2889.data.channel10 = true;
                el2889.data.channel11 = true;
                el2889.data.channel12 = true;
                el2889.data.channel13 = true;
                el2889.data.channel14 = true;
                el2889.data.channel15 = true;
                el2889.data.channel16 = true;
            }
            el2889.Write_Data();

            stop_change_time = stop_change_time - 1;
            if(stop_change_time == 0){
                state = !state;
                stop_change_time = 500;
            }
        }
        ecrt_domain_queue(el2889.domain); // 根据domain数据生成帧
        ecrt_master_send(Master.master);  // EtherCAT主站发送
    }
    Master.EtherCAT_Master_Deactivate();
}