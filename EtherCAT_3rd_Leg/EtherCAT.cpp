#include "EtherCAT.h"
#include <array>

bool EtherCAT_Running_Flag = true;
static void EtherCAT_Exit(int signal) { EtherCAT_Running_Flag = false; }

static EtherCAT_Master Master;
// TODO: 该处的SDO参数配置需要根据实际情况修改
static ELMO_SDO_Data SDO_Data_Larger = {1, 1, 13, 2, 131072, 10000, 1, 1, 0, CSP};
static ELMO_SDO_Data SDO_Data_Smaller = {1, 1, 5, 1, 131072, 10000, 1, 1, 0, CSP};
static std::array<ELMO *, 8> Leg_Elmos;

// SharedDataConfig sharedDataConfig;
// static SharedMemory LinerMotorCmdMemory = SharedMemory(sharedDataConfig.FILE_PATH, sharedDataConfig.length[1], sharedDataConfig.keyNum[1], 0, sharedDataConfig.semName[1]);;
// static SharedMemory LinerMotorStateMemory = SharedMemory(sharedDataConfig.FILE_PATH, sharedDataConfig.length[4], sharedDataConfig.keyNum[4], 0, sharedDataConfig.semName[4]);
// static AllLineMotorsCmdData allLineMotorsCmdData;
// static AllLineMotorsStateData allLineMotorsStateData;

static int position = 0;
static uint16_t printCount = PRINT_COUNT;
static struct timespec cycleTime = {0, TASK_PERIOD_NS};
static struct timespec wakeupTime;
static struct timespec endTime;
static struct timespec actualTime;
static struct timespec actualPeriodTime;


static void Elmo_Display_Function(ELMO& _Elmo, int& _Position)
{
    if(_Elmo.data.drive_state == dsOperationEnabled)
    {
        ETHERCAT_INFO("Elmo%d: act position = %d, status = 0x%x, drive_state = %d, drive_mode = %d", \
        _Position, _Elmo.domain_data.TxPDO.actual_position, _Elmo.domain_data.TxPDO.status_word, _Elmo.data.drive_state, _Elmo.data.drive_mode);
    }
}

static void Elmo_Move_Function(ELMO& _Elmo, int& _Position)
{
    if (_Elmo.data.reset_busy == true)
      {
          _Elmo.Reset_Motor_State();
      }
      // 急停
      else if(_Elmo.data.quickstop_busy == true)
      {
          _Elmo.Motor_Quickstop();
      }
      // 使能
      else if (_Elmo.data.power_busy == true)
      {
          _Elmo.Enable_Motor(CSP);
      }

    // * 用于急停自动复位
    // else if(_Elmo.data.drive_state == dsSwitchOnDisabled)
    // {
    //     _Elmo.Reset_Motor_State();
    // }

    // * 检测没有处于复位、急停或者使能状态，则进行运动逻辑
    if(_Elmo.data.drive_state == dsOperationEnabled && _Elmo.data.reset_busy == false && _Elmo.data.power_busy == false && _Elmo.data.quickstop_busy == false)
    {
        if (_Elmo.data.drive_mode  == CSP)
        { // 位置模式
            if (_Elmo.data.home_busy == true)
            {
                _Elmo.Move_Motor_Home();
            }
            else
            {
              // TODO: 运动逻辑尚未完善
                if (_Elmo.data.position_move_enable == true)
                {// 开始运动
                    // TODO：此处放置运动逻辑
                      // _Elmo.domain_data.RxPDO.target_position = (int32_t)(allLineMotorsCmdData.kneeMotorsCmd[0].linearPos * 10000);
                    _Elmo.domain_data.RxPDO.target_position = _Elmo.domain_data.TxPDO.actual_position;
                }
                else
                {
                    _Elmo.domain_data.RxPDO.target_position = _Elmo.domain_data.TxPDO.actual_position;
                }
                EC_WRITE_S32(_Elmo.domain_pd + _Elmo.domain_offset.RxPDO.target_position, _Elmo.domain_data.RxPDO.target_position);
            }
        }
    }
}

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

        // TODO：此处需要更改共享内存的读取和写入逻辑
        // 从共享内存读取直线电缸控制数据
        // LinerMotorCmdMemory.read(&allLineMotorsCmdData, 1, 0);
        // // 写入直线电机状态至共享内存中
        // allLineMotorsStateData.kneeMotorsState[0].linearPos = (float)(Elmo1.domain_data.TxPDO.actual_position) / 10000;
        // // allLineMotorsStateData.kneeMotorsState[0].linearSpeed = (float)(Elmo1.domain_data.TxPDO.actual_velocity) / 10000;
        // allLineMotorsStateData.kneeMotorsState[0].linearTau = \
        // (((float)(Elmo1.domain_data.TxPDO.torque_input) / 1000) - 0.023) / 3.630 * 15000;
        // allLineMotorsStateData.kneeMotorsState[1].linearPos = (float)(Elmo2.domain_data.TxPDO.actual_position) / 10000;
        // // allLineMotorsStateData.kneeMotorsState[1].linearSpeed = (float)(Elmo2.domain_data.TxPDO.actual_velocity) / 10000;
        // allLineMotorsStateData.kneeMotorsState[1].linearTau = \
        // (((float)(Elmo2.domain_data.TxPDO.torque_input) / 1000) - 0.028) / 3.590 * 15000;
        // LinerMotorStateMemory.write(&allLineMotorsStateData, 1, 0);


#ifdef ETHERCAT_DEBUG_MODE
        Master.EtherCAT_Check_States();

        position = 0;
        for(auto &elmo : Leg_Elmos)
        {
            elmo->Check_Motor_State(position);
            ++position;
        }
#endif

        // ***************** ELMO输出函数  ********************
        if(printCount == 0)
        {
            position = 0;
            for(auto &elmo : Leg_Elmos)
            {
                Elmo_Display_Function(*elmo, position);
                ++position;
            }
            printCount = PRINT_COUNT;
        }
        else
        {
            printCount--;
        }
        // ***************** ELMO输出函数  ********************


        // ***************** ELMO运动函数  ********************

        position = 0; // NOTICE：该处预留了位置变量，ELMO运动函数内部的共享内存读取逻辑
        for(auto &elmo : Leg_Elmos)
        {
            Elmo_Move_Function(*elmo, position);
            ++position;
        }

        // ***************** ELMO运动函数  ********************



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
    // LinerMotorCmdMemory.disconnect();
    // LinerMotorStateMemory.disconnect();
    return 0;
}

void EtherCAT_Start()
{
    Leg_Elmos[0] = new ELMO(Master, 0, 0, SDO_Data_Smaller);  // 左腿踝关节右侧
    Leg_Elmos[1] = new ELMO(Master, 0, 1, SDO_Data_Larger);   // 左腿膝关节
    Leg_Elmos[2] = new ELMO(Master, 0, 2, SDO_Data_Larger);   // 左腿髋关节
    Leg_Elmos[3] = new ELMO(Master, 0, 3, SDO_Data_Smaller);  // 左腿踝关节左侧
    Leg_Elmos[4] = new ELMO(Master, 0, 4, SDO_Data_Smaller);  // 右腿踝关节左侧
    Leg_Elmos[5] = new ELMO(Master, 0, 5, SDO_Data_Larger);   // 右腿膝关节
    Leg_Elmos[6] = new ELMO(Master, 0, 6, SDO_Data_Larger);   // 右腿髋关节
    Leg_Elmos[7] = new ELMO(Master, 0, 7, SDO_Data_Smaller);  // 右腿踝关节右侧

    Master.EtherCAT_Master_Config();
    ETHERCAT_MESSAGE("*EtherCAT Master and Slaves config success*");
    signal(SIGINT, EtherCAT_Exit);

    // LinerMotorCmdMemory.connect();
    // LinerMotorStateMemory.connect();
    // allLineMotorsCmdData.kneeMotorsCmd[0].linearPos = 0;
    // allLineMotorsCmdData.kneeMotorsCmd[1].linearPos = 0;
    // allLineMotorsStateData.kneeMotorsState[0].motorId = 0;
    // allLineMotorsStateData.kneeMotorsState[0].linearPos = 0;
    // allLineMotorsStateData.kneeMotorsState[0].linearTau = 0;
    // allLineMotorsStateData.kneeMotorsState[1].motorId = 1;
    // allLineMotorsStateData.kneeMotorsState[1].linearPos = 0;
    // allLineMotorsStateData.kneeMotorsState[1].linearTau = 0;

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

    int result = pthread_join(thread1, nullptr);

    position = 0;
    for(auto &elmo : Leg_Elmos)
    {
        delete elmo;
        ++position;
    }
}