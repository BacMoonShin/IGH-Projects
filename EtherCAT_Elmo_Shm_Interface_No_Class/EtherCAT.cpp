#include "EtherCAT.h"

#define _ELMO_1_ENABLE_
#define _ELMO_2_ENABLE_

bool EtherCAT_Running_Flag = true;
static void EtherCAT_Exit(int signal) { EtherCAT_Running_Flag = false; }

static EtherCAT_Master Master;
static ELMO_SDO_Data SDO_Data = {1, 1, 3, 2, 131072, 10000, 1, 1, 0, CSP};

#ifdef _ELMO_1_ENABLE_
static ELMO Elmo1(Master, 0, 0, SDO_Data);
#endif
#ifdef _ELMO_2_ENABLE_
static ELMO Elmo2(Master, 0, 1, SDO_Data);
#endif

SharedDataConfig sharedDataConfig;
static SharedMemory LinerMotorCmdMemory = SharedMemory(sharedDataConfig.FILE_PATH, sharedDataConfig.length[1], sharedDataConfig.keyNum[1], 0, sharedDataConfig.semName[1]);
static SharedMemory LinerMotorStateMemory = SharedMemory(sharedDataConfig.FILE_PATH, sharedDataConfig.length[4], sharedDataConfig.keyNum[4], 0, sharedDataConfig.semName[4]);
static AllLineMotorsCmdData allLineMotorsCmdData;
static AllLineMotorsStateData allLineMotorsStateData;

static uint16_t printCount = PRINT_COUNT;
// uint16_t UpdateCount = UPDATE_COUNT;
static struct timespec cycleTime = {0, TASK_PERIOD_NS};
static struct timespec wakeupTime;
static struct timespec endTime;
static struct timespec actualTime;
static struct timespec actualPeriodTime;

static void* EtherCAT_Thread(void*)
{
    clock_gettime(CLOCK_REALTIME, &wakeupTime);
    while (EtherCAT_Running_Flag)
    {
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &wakeupTime, NULL);
#ifdef ENABLE_DC
        ecrt_master_application_time(Master.master, TIMESPEC2NS(wakeupTime));
        ecrt_master_sync_reference_clock(Master.master);
#endif

        // 接收过程数据
        Master.EtherCAT_Master_Receive();
        // 从共享内存读取直线电缸控制数据
        LinerMotorCmdMemory.read(&allLineMotorsCmdData, 1, 0);
        // 写入直线电机状态至共享内存中
        allLineMotorsStateData.kneeMotorsState[0].linearPos = (float)(Elmo1.domain_data.TxPDO.actual_position) / 10000;
        // allLineMotorsStateData.kneeMotorsState[0].linearSpeed = (float)(Elmo1.domain_data.TxPDO.actual_velocity) / 10000;
        allLineMotorsStateData.kneeMotorsState[0].linearTau = \
        (((float)(Elmo1.domain_data.TxPDO.torque_input) / 1000) - 0.023) / 3.630 * 15000;
        allLineMotorsStateData.kneeMotorsState[1].linearPos = (float)(Elmo2.domain_data.TxPDO.actual_position) / 10000;
        // allLineMotorsStateData.kneeMotorsState[1].linearSpeed = (float)(Elmo2.domain_data.TxPDO.actual_velocity) / 10000;
        allLineMotorsStateData.kneeMotorsState[1].linearTau = \
        (((float)(Elmo2.domain_data.TxPDO.torque_input) / 1000) - 0.028) / 3.590 * 15000;
        LinerMotorStateMemory.write(&allLineMotorsStateData, 1, 0);


#ifdef ETHERCAT_DEBUG_MODE
        Master.EtherCAT_Check_States();
#ifdef _ELMO_1_ENABLE_
        Elmo1.Check_Motor_State(1);
#endif
#ifdef _ELMO_2_ENABLE_
        Elmo2.Check_Motor_State(2);
#endif
#endif
        if(printCount == 0)
        {
#ifdef _ELMO_1_ENABLE_
            if(Elmo1.data.drive_state == dsOperationEnabled)
            {
                ETHERCAT_INFO("Elmo1: act position = %d, act torque = %d, status = 0x%x, drive_state = %d, drive_mode = %d",
                              Elmo1.domain_data.TxPDO.actual_position, Elmo1.domain_data.TxPDO.torque_input, Elmo1.domain_data.TxPDO.status_word, Elmo1.data.drive_state, Elmo1.data.drive_mode);
            }
#endif
#ifdef _ELMO_2_ENABLE_
            if(Elmo2.data.drive_state == dsOperationEnabled)
            {
                ETHERCAT_INFO("Elmo2: act position = %d, act torque = %d, status = 0x%x, drive_state = %d, drive_mode =%d",
                              Elmo2.domain_data.TxPDO.actual_position, Elmo2.domain_data.TxPDO.torque_input, Elmo2.domain_data.TxPDO.status_word, Elmo2.data.drive_state, Elmo2.data.drive_mode);
            }
#endif
            printCount = PRINT_COUNT;
        }
        else
        {
            printCount--;
        }

#ifdef _ELMO_1_ENABLE_
        /* ***************** ELMO 1 ******************** */
        // 轮询检测是否在进行复位、急停或者使能
        // 复位或急停
        if (Elmo1.data.reset_busy == true)
        {
            Elmo1.Reset_Motor_State();
        }
        // 急停
        else if(Elmo1.data.quickstop_busy == true)
        {
            Elmo1.Motor_Quickstop();
        }
        // 使能
        else if (Elmo1.data.power_busy == true)
        {
            Elmo1.Enable_Motor(CSP);
        }

        else if(Elmo1.data.drive_state == dsSwitchOnDisabled)
        {
            Elmo1.Reset_Motor_State();
        }

        // 检测没有处于复位、急停或者使能状态，则进行运动逻辑
        if(Elmo1.data.drive_state == dsOperationEnabled && Elmo1.data.reset_busy == false && Elmo1.data.power_busy == false && Elmo1.data.quickstop_busy == false)
        {
            if (Elmo1.data.drive_mode  == CSP)
            { // 位置模式
                if (Elmo1.data.home_busy == true)
                {
                    Elmo1.Move_Motor_Home();
                }
                else
                {
                    if (Elmo1.data.position_move_enable == true)
                    {// 开始运动
                        Elmo1.domain_data.RxPDO.target_position = (int32_t)(allLineMotorsCmdData.kneeMotorsCmd[0].linearPos * 10000);
                    }
                    else
                    {
                        Elmo1.domain_data.RxPDO.target_position = Elmo1.domain_data.TxPDO.actual_position;
                    }
                    EC_WRITE_S32(Elmo1.domain_pd + Elmo1.domain_offset.RxPDO.target_position, Elmo1.domain_data.RxPDO.target_position);
                }
            }
        }
        /* ***************** ELMO 1 ******************** */
#endif

#ifdef _ELMO_2_ENABLE_
        /* ***************** ELMO 2 ******************** */
        // 轮询检测是否在进行复位、急停或者使能
        // 复位或急停
        if (Elmo2.data.reset_busy == true)
        {
            Elmo2.Reset_Motor_State();
        }
        // 急停
        else if(Elmo2.data.quickstop_busy == true)
        {
            Elmo2.Motor_Quickstop();
        }
        // 使能
        else if (Elmo2.data.power_busy == true)
        {
            Elmo2.Enable_Motor(CSP);
        }

        else if(Elmo2.data.drive_state == dsSwitchOnDisabled)
        {
            Elmo2.Reset_Motor_State();
        }

        // 检测没有处于复位、急停或者使能状态，则进行运动逻辑
        if(Elmo2.data.drive_state == dsOperationEnabled && Elmo2.data.reset_busy == false && Elmo2.data.power_busy == false && Elmo2.data.quickstop_busy == false)
        {
            if (Elmo2.data.drive_mode  == CSP)
            { // 位置模式
                if (Elmo2.data.home_busy == true)
                {
                    Elmo2.Move_Motor_Home();
                }
                else
                {
                    if (Elmo2.data.position_move_enable == true)
                    {// 开始运动
                        Elmo2.domain_data.RxPDO.target_position = (int32_t)(allLineMotorsCmdData.kneeMotorsCmd[1].linearPos * 10000);
                    }
                    else
                    {
                        Elmo2.domain_data.RxPDO.target_position = Elmo2.domain_data.TxPDO.actual_position;
                    }
                    EC_WRITE_S32(Elmo2.domain_pd + Elmo2.domain_offset.RxPDO.target_position, Elmo2.domain_data.RxPDO.target_position);
                }
            }
        }
        /* ***************** ELMO 2 ******************** */
#endif


#ifdef ENABLE_DC
        clock_gettime(CLOCK_REALTIME, &actualTime);
        ecrt_master_sync_reference_clock_to(Master.master, TIMESPEC2NS(actualTime));
        ecrt_master_sync_slave_clocks(Master.master);
#endif

        Master.EtherCAT_Master_Send();

        clock_gettime(CLOCK_REALTIME, &endTime);
        actualPeriodTime = timespec_minus(endTime, wakeupTime);
        if(TIMESPEC2NS(actualPeriodTime) > TASK_PERIOD_NS)
        {
            ETHERCAT_WARNING("ActualPeriodTime is larger than task period!");
            // wakeupTime = timespec_add(wakeupTime, cycleTime);
            wakeupTime = endTime;
        }
        else
        {
            // ETHERCAT_INFO("actualPeriodTime is %d ns", TIMESPEC2NS(actualPeriodTime));
            wakeupTime = timespec_add(wakeupTime, cycleTime);
        }
    }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &wakeupTime, NULL);
    Master.EtherCAT_Master_Exit();
    LinerMotorCmdMemory.disconnect();
    LinerMotorStateMemory.disconnect();
}

void EtherCAT_Start()
{
    int res = system("./SetPREOP.sh");
    if(res == -1 || res == 127){ETHERCAT_CRITICAL("Reset Elmo to PREOP failed!");}

    Master.EtherCAT_Master_Config();
    ETHERCAT_MESSAGE("*EtherCAT Master and Slaves config success*");
    signal(SIGINT, EtherCAT_Exit);

    LinerMotorCmdMemory.connect();
    LinerMotorStateMemory.connect();
    allLineMotorsCmdData.kneeMotorsCmd[0].linearPos = 0;
    allLineMotorsCmdData.kneeMotorsCmd[1].linearPos = 0;
    allLineMotorsStateData.kneeMotorsState[0].motorId = 0;
    allLineMotorsStateData.kneeMotorsState[0].linearPos = 0;
    allLineMotorsStateData.kneeMotorsState[0].linearTau = 0;
    allLineMotorsStateData.kneeMotorsState[1].motorId = 1;
    allLineMotorsStateData.kneeMotorsState[1].linearPos = 0;
    allLineMotorsStateData.kneeMotorsState[1].linearTau = 0;

    pthread_t thread1;
    cpu_set_t cpuset;
    struct sched_param param1;

     // 创建线程1
    pthread_create(&thread1, nullptr, EtherCAT_Thread, NULL);

    // 设置线程1的优先级
    param1.sched_priority = 20;                        // 设置线程1优先级为10
    pthread_setschedparam(thread1, SCHED_RR, &param1); // pthread_setschedparam函数针对线程优先级进行设置，SCHED_RR表示将根据优先级使用轮转的实时调度策略，允许同优先级的线程进行轮转运行

    // 设置线程1的CPU亲和性（绑定到CPU 0）
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset); // 设置线程1只能在CPU 0上运行
    pthread_setaffinity_np(thread1, sizeof(cpu_set_t), &cpuset);

    pthread_join(thread1, nullptr);
}