/*
 * @Author: your name
 * @Date: 2024-09-24 14:35:04
 * @LastEditors: your name
 * @LastEditTime: 2024-09-24 19:30:04
 * @Description:
 * @FilePath: \EtherCAT\EtherCAT_Demo_V0.1.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>


// ethercat主站应用程序所用的参数
#define TASK_FREQUENCY 1000                                            //*Hz* 任务周期（任务频率）
#define ENCODER_RESOLUTION 131072                                    // 编码器分辨率
#define HOME_VECOLITY 5                                              // r/s，回零速度
#define HOME_STEP HOME_VECOLITY *ENCODER_RESOLUTION / TASK_FREQUENCY // pulse 回零步长 = 回零速度 * 编码器分辨率 * 任务周期
#define POSITION_STEP 1 / TASK_FREQUENCY                             // 位置模式下步长
#define Pi 3.141592654                                               // 圆周率

// 从站配置所用的参数XQ
#define SLAVEPOS 0, 2              // 迈信伺服EP3E在ethercat总线上的位置
#define EL2889 0x00000002, 0x0b493052 // EP3E的厂家标识和产品标识

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

// 迈信伺服驱动器里PDO入口的偏移量
/* 我们需要定义一些变量去关联需要用到的从站的PD0对象*/
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

// 迈信伺服电机结构体
struct MOTOR
{
    // 关于ethercat master
    ec_master_t *master;            // 主站
    ec_master_state_t master_state; // 主站状态

    ec_domain_t *domain;            // 域
    ec_domain_state_t domain_state; // 域状态

    ec_slave_config_t *maxsine_EP3E;            // 从站配置，这里只有一台迈信伺服
    ec_slave_config_state_t maxsine_EP3E_state; // 从站配置状态

    uint8_t *domain_pd;                    // Process Data
    struct DRIVERVARIABLE drive_variables; // 从站驱动器变量

    int32_t targetPosition;  // 电机的目标位置
    int8_t opModeSet;        // 电机运行模式的设定值,默认位置模式
    int8_t opmode;           // 驱动器当前运行模式
    int32_t currentVelocity; // 电机当前运行速度
    int32_t currentPosition; // 电机当前位置
    uint16_t status;         // 驱动器状态字

    enum DRIVERSTATE driveState; // 驱动器状态

    // 关于电机控制的私有变量
    bool powerBusy;      // 使能标志位
    bool resetBusy;      // 复位标志位
    bool quickStopBusy;  // 急停标志位
    bool homeBusy;       // 回零标志位
    bool positionMoving; // 位置模式下运动
};

// 倍福输出模块结构体
struct OTModule
{
    // 关于ethercat master
    ec_master_t *master;            // 主站
    ec_master_state_t master_state; // 主站状态

    ec_domain_t *domain;            // 域
    ec_domain_state_t domain_state; // 域状态

    ec_slave_config_t *BackHoff_EL2889;            // 从站配置，这里只有一台迈信伺服
    ec_slave_config_state_t BackHoff_EL2889_State; // 从站配置状态

    uint8_t *domain_pd;                    // Process Data
    struct MODULEVARIABLE drive_variables; // 从站驱动器变量


    // 定义每一个通道的控制量
    bool channel1;
    bool channel2;
    bool channel3;
    bool channel4;
    bool channel5;
    bool channel6;
    bool channel7;
    bool channel8;
    bool channel9;
    bool channel10;
    bool channel11;
    bool channel12;
    bool channel13;
    bool channel14;
    bool channel15;
    bool channel16;
};

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

// 初始化EtherCAT主站函数
static void init_EtherCAT_master(struct OTModule *module)
{
    // 变量与对应PDO数据对象关联
    // 域domain可以看成将接收到的报文保存在domain中，需要通过注册PDO entry来获得每一个对象字典对应的PDO数据对象的偏移量，从而进行访问
    ec_pdo_entry_reg_t domain_regs[] = {
        /* {从站在总线上的位置，迈信伺服的厂家标识和产品标识，PDO数据对象在对象字典中对应的主索引，PDO数据对象在对象字典中的子索引，用于保存对应数据对象访问偏移量的变量} */
        {SLAVEPOS, EL2889, 0x7000, 0x01, &module->drive_variables.channel1, &module->drive_variables.channel1_offset},
        {SLAVEPOS, EL2889, 0x7010, 0x01, &module->drive_variables.channel2, &module->drive_variables.channel2_offset},
        {SLAVEPOS, EL2889, 0x7020, 0x01, &module->drive_variables.channel3, &module->drive_variables.channel3_offset},
        {SLAVEPOS, EL2889, 0x7030, 0x01, &module->drive_variables.channel4, &module->drive_variables.channel4_offset},
        {SLAVEPOS, EL2889, 0x7040, 0x01, &module->drive_variables.channel5, &module->drive_variables.channel5_offset},
        {SLAVEPOS, EL2889, 0x7050, 0x01, &module->drive_variables.channel6, &module->drive_variables.channel6_offset},
        {SLAVEPOS, EL2889, 0x7060, 0x01, &module->drive_variables.channel7, &module->drive_variables.channel7_offset},
        {SLAVEPOS, EL2889, 0x7070, 0x01, &module->drive_variables.channel8, &module->drive_variables.channel8_offset},
        {SLAVEPOS, EL2889, 0x7080, 0x01, &module->drive_variables.channel9, &module->drive_variables.channel9_offset},
        {SLAVEPOS, EL2889, 0x7090, 0x01, &module->drive_variables.channel10, &module->drive_variables.channel10_offset},
        {SLAVEPOS, EL2889, 0x70a0, 0x01, &module->drive_variables.channel11, &module->drive_variables.channel11_offset},
        {SLAVEPOS, EL2889, 0x70b0, 0x01, &module->drive_variables.channel12, &module->drive_variables.channel12_offset},
        {SLAVEPOS, EL2889, 0x70c0, 0x01, &module->drive_variables.channel13, &module->drive_variables.channel13_offset},
        {SLAVEPOS, EL2889, 0x70d0, 0x01, &module->drive_variables.channel14, &module->drive_variables.channel14_offset},
        {SLAVEPOS, EL2889, 0x70e0, 0x01, &module->drive_variables.channel15, &module->drive_variables.channel15_offset},
        {SLAVEPOS, EL2889, 0x70f0, 0x01, &module->drive_variables.channel16, &module->drive_variables.channel16_offset},
        {}};

    // 填充相关PDOS信息
    // 定义向RxPDO以及TxPDO的映射表中填充映射对象信息，ec_pdo_entry_info_t（PDO映射信息）格式为 {映射对象主索引，映射对象子索引，映射对象的大小}，单个信息为32bit（16+8+8）
    ec_pdo_entry_info_t EP3E_pdo_entries[] = {/*RxPdo 0x1600*/
                                              {CTRL_WORD, 0x00, 16},
                                              {OPERATION_MODE, 0x00, 8},
                                              {TARGET_VELOCITY, 0x00, 32},
                                              {TARGET_POSITION, 0x00, 32},
                                              /*TxPdo 0x1A00*/
                                              {STATUS_WORD, 0x00, 16},
                                              {MODE_DISPLAY, 0x00, 8},
                                              {CURRENT_VELOCITY, 0x00, 32},
                                              {CURRENT_POSITION, 0x00, 32}};
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

    // ec_pdo_info_t 用于初始化RxPDO以及TxPDO的映射表信息，结构为{RxPDO或TxPDO的对象字典索引，映射个数，ec_pdo_entry_info_t中RxPDO或TxPDO映射信息对应起始位}
    ec_pdo_info_t EP3E_pdos[] = {// RxPdo
                                 {RXPDO, 4, EP3E_pdo_entries + 0},
                                 // TxPdo
                                 {TXPDO, 4, EP3E_pdo_entries + 4}};
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
                                   {0x160f, 1, EL2889_pdo_entries + 15},/* Channel 16 */};

    // ec_sync_info_t {通道编号，工作模式，通信目标数？，RxPDO或TxPDO映射表索引位置，看门狗模式？}
    ec_sync_info_t EP3E_syncs[] = {{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                                   {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                                   {2, EC_DIR_OUTPUT, 1, EP3E_pdos + 0, EC_WD_DISABLE},
                                   {3, EC_DIR_INPUT, 1, EP3E_pdos + 1, EC_WD_DISABLE},
                                   {0xFF}};
    ec_sync_info_t EL2889_syncs[] = {
                                    {0, EC_DIR_OUTPUT, 8, EL2889_pdos + 0, EC_WD_ENABLE},
                                    {1, EC_DIR_OUTPUT, 8, EL2889_pdos + 8, EC_WD_ENABLE},
                                    {0xff}};

    // 创建ethercat主站master
    module->master = ecrt_request_master(0);
    if (!module->master)
    {
        printf("Failed to create ethercat master!\n");
        exit(EXIT_FAILURE); // 创建失败，退出线程
    }

    // 创建域domain
    module->domain = ecrt_master_create_domain(module->master);
    if (!module->domain)
    {
        printf("Failed to create master domain!\n");
        exit(EXIT_FAILURE); // 创建失败，退出线程
    }

    // 配置从站
    if (!(module->BackHoff_EL2889 = ecrt_master_slave_config(module->master, SLAVEPOS, EL2889)))
    {
        printf("Failed to get slave configuration for EL2889!\n");
        exit(EXIT_FAILURE); // 配置失败，退出线程
    }

    // 对从站进行配置PDOs
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(module->BackHoff_EL2889, EC_END, EL2889_syncs))
    {
        printf("Failed to configure EL2889 PDOs!\n");
        exit(EXIT_FAILURE); // 配置失败，退出线程
    }
    else
    {
        printf("*Success to configuring EL2889 PDOs*\n"); // 配置成功
    }

    // 注册PDO entry
    if (ecrt_domain_reg_pdo_entry_list(module->domain, domain_regs))
    {
        printf("PDO entry registration failed!\n");
        exit(EXIT_FAILURE); // 注册失败，退出线程
    }
    else
    {
        printf("*Success to configuring EL2889 PDO entry*\n"); // 注册成功
    }

    // 激活主站master
    printf("Activating master...\n");
    if (ecrt_master_activate(module->master))
    {
        exit(EXIT_FAILURE); // 激活失败，退出线程
    }
    else
    {
        printf("*Master activated*\n"); // 激活成功
    }

    // 从domain与获取过程数据
    if (!(module->domain_pd = ecrt_domain_data(module->domain)))
    {
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char **argv)
{
    struct OTModule el2889;
    init_EtherCAT_master(&el2889);
    printf("*It's working now*\n");

    int stop_change_time = 500;
    int stop_print_time = 250;
    bool state = false;
    while (true)
    {
        usleep(1*1000*1000/TASK_FREQUENCY); // 计算一次周期的时间

        // 接收过程数据
        ecrt_master_receive(el2889.master); // EtherCAT 主站接收一次报文
        ecrt_domain_process(el2889.domain); // 可以理解成根据报文更新一次domain域中的信息

        // 检查过程数据状态（可选）
        check_domain_state(el2889.domain, &el2889.domain_state);
        // 检查主站状态
        check_master_state(el2889.master, &el2889.master_state);
        // 检查从站配置状态
        check_slave_config_states(el2889.BackHoff_EL2889, &el2889.BackHoff_EL2889_State);

        if(!(el2889.BackHoff_EL2889_State.operational)){
            printf("*Waiting EL2889 to become operational*\n");
        }
        else{
            // 读取数据，根据注册获得的PDO对象字典的偏移量进行读取
            el2889.channel1 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel1, el2889.drive_variables.channel1_offset); // 状态字
            el2889.channel2 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel2, el2889.drive_variables.channel2_offset); // 状态字
            el2889.channel3 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel3, el2889.drive_variables.channel3_offset); // 状态字
            el2889.channel4 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel4, el2889.drive_variables.channel4_offset); // 状态字
            el2889.channel5 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel5, el2889.drive_variables.channel5_offset); // 状态字
            el2889.channel6 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel6, el2889.drive_variables.channel6_offset); // 状态字
            el2889.channel7 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel7, el2889.drive_variables.channel7_offset); // 状态字
            el2889.channel8 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel8, el2889.drive_variables.channel8_offset); // 状态字
            el2889.channel9 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel9, el2889.drive_variables.channel9_offset); // 状态字
            el2889.channel10 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel10, el2889.drive_variables.channel10_offset); // 状态字
            el2889.channel11 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel11, el2889.drive_variables.channel11_offset); // 状态字
            el2889.channel12 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel12, el2889.drive_variables.channel12_offset); // 状态字
            el2889.channel13 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel13, el2889.drive_variables.channel13_offset); // 状态字
            el2889.channel14 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel14, el2889.drive_variables.channel14_offset); // 状态字
            el2889.channel15 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel15, el2889.drive_variables.channel15_offset); // 状态字
            el2889.channel16 =
                EC_READ_BIT(el2889.domain_pd + el2889.drive_variables.channel16, el2889.drive_variables.channel16_offset); // 状态字

            stop_print_time = stop_print_time - 1;
            if(stop_print_time == 0){
              printf("EL2889:  channel0 = %d , channel1 = %d , channel2 = %d , channel3 = %d , channel4 = %d , channel5 = %d , channel6 = %d , channel7 = %d , channel8 = %d , channel9 = %d , channel10 = %d , channel11 = %d , channel12 = %d , channel13 = %d , channel14 = %d , channel15 = %d\n",
                     el2889.channel1, el2889.channel2, el2889.channel3, el2889.channel4, el2889.channel5, el2889.channel6, el2889.channel7, el2889.channel8, el2889.channel9, el2889.channel10, el2889.channel11, el2889.channel12, el2889.channel13, el2889.channel14, el2889.channel15, el2889.channel16);
              stop_print_time = 250;
            }

            if(state){
                el2889.channel1 = true;
                el2889.channel2 = true;
                el2889.channel3 = true;
                el2889.channel4 = true;
                el2889.channel5 = true;
                el2889.channel6 = true;
                el2889.channel7 = true;
                el2889.channel8 = true;
                el2889.channel9 = false;
                el2889.channel10 = false;
                el2889.channel11 = false;
                el2889.channel12 = false;
                el2889.channel13 = false;
                el2889.channel14 = false;
                el2889.channel15 = false;
                el2889.channel16 = false;
            }
            else{
                el2889.channel1 = false;
                el2889.channel2 = false;
                el2889.channel3 = false;
                el2889.channel4 = false;
                el2889.channel5 = false;
                el2889.channel6 = false;
                el2889.channel7 = false;
                el2889.channel8 = false;
                el2889.channel9 = true;
                el2889.channel10 = true;
                el2889.channel11 = true;
                el2889.channel12 = true;
                el2889.channel13 = true;
                el2889.channel14 = true;
                el2889.channel15 = true;
                el2889.channel16 = true;
            }
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel1, el2889.drive_variables.channel1_offset, el2889.channel1);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel2, el2889.drive_variables.channel2_offset, el2889.channel2);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel3, el2889.drive_variables.channel3_offset, el2889.channel3);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel4, el2889.drive_variables.channel4_offset, el2889.channel4);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel5, el2889.drive_variables.channel5_offset, el2889.channel5);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel6, el2889.drive_variables.channel6_offset, el2889.channel6);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel7, el2889.drive_variables.channel7_offset, el2889.channel7);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel8, el2889.drive_variables.channel8_offset, el2889.channel8);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel9, el2889.drive_variables.channel9_offset, el2889.channel9);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel10, el2889.drive_variables.channel10_offset, el2889.channel10);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel11, el2889.drive_variables.channel11_offset, el2889.channel11);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel12, el2889.drive_variables.channel12_offset, el2889.channel12);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel13, el2889.drive_variables.channel13_offset, el2889.channel13);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel14, el2889.drive_variables.channel14_offset, el2889.channel14);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel15, el2889.drive_variables.channel15_offset, el2889.channel15);
            EC_WRITE_BIT(el2889.domain_pd + el2889.drive_variables.channel16, el2889.drive_variables.channel16_offset, el2889.channel16);

            stop_change_time = stop_change_time - 1;
            if(stop_change_time == 0){
                state = !state;
                stop_change_time = 500;
            }
        }
        ecrt_domain_queue(el2889.domain); // 根据domain数据生成帧
        ecrt_master_send(el2889.master);  // EtherCAT主站发送
    }
    ecrt_master_deactivate(el2889.master);
}