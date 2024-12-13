#include <stdio.h>
#include <stdlib.h>
#include <ecrt.h>
#include <math.h>

// ethercat主站应用程序所用的参数
#define TASK_FREQUENCY 4000                                          // *Hz* 任务周期（任务频率）
#define ENCODER_RESOLUTION 131072                                    // 编码器分辨率
#define HOME_VECOLITY 5                                              // r/s，回零速度
#define HOME_STEP HOME_VECOLITY *ENCODER_RESOLUTION / TASK_FREQUENCY // pulse 回零步长 = 回零速度 * 编码器分辨率 * 任务周期
#define POSITION_STEP 1 / TASK_FREQUENCY                             // 位置模式下步长
#define Pi 3.141592654                                               // 圆周率

// 从站配置所用的参数XQ
#define EP3ESLAVEPOS 0, 0              // 迈信伺服EP3E在ethercat总线上的位置
#define MAXSINE 0x000007DD, 0x00000001 // EP3E的厂家标识和产品标识

// CoE对象字典
#define RXPDO 0x1600 // RxPDO的映射表在对象字典中的索引位置
#define TXPDO 0x1A00 // TxPDO的映射表在对象字典中的索引位置

/*CiA 402数据对象(Data Object)*/
#define CTRL_WORD 0x6040        // 控制字的数据对象
#define STATUS_WORD 0x6041      // 状态字的数据对象
#define OPERATION_MODE 0x6060   // 设定运行模式的数据对象
#define MODE_DISPLAY 0x6061     // 当前运行模式的数据对象
#define ACUTAL_POSITION 0x6064 // 当前位置的数据对象
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
  unsigned int ctrl_word;        // 控制字
  unsigned int operation_mode;   // 设定运行模式
  unsigned int target_velocity;  // 目标速度 （pulse/s)
  unsigned int target_postion;   // 目标位置 （pulse）
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
  struct DRIVERVARIABLE drive_variables; // 从站驱动器变量指针偏移位结构体

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
    printf("EP3E: State 0x%02X.\n", s.al_state);
  }
  if (s.online != slave_state->online)
  {
    printf("EP3E: %s.\n", s.online ? "online" : "offline");
  }
  if (s.operational != slave_state->operational)
  {
    printf("EP3E: %soperational.\n", s.operational ? "" : "Not ");
  }
  *slave_state = s;
}

// 初始化EtherCAT主站函数
static void
init_EtherCAT_master(struct MOTOR *motor)
{
  // 变量与对应PDO数据对象关联
  // 域domain可以看成将接收到的报文保存在domain中，需要通过注册PDO entry来获得每一个对象字典对应的PDO数据对象的偏移量，从而进行访问
  ec_pdo_entry_reg_t domain_regs[] = {
      /* {从站在总线上的位置，迈信伺服的厂家标识和产品标识，PDO数据对象在对象字典中对应的主索引，PDO数据对象在对象字典中的子索引，用于保存对应数据对象访问偏移量的变量} */
      {EP3ESLAVEPOS, MAXSINE, CTRL_WORD, 0, &motor->drive_variables.ctrl_word},
      {EP3ESLAVEPOS, MAXSINE, OPERATION_MODE, 0, &motor->drive_variables.operation_mode},
      {EP3ESLAVEPOS, MAXSINE, TARGET_VELOCITY, 0, &motor->drive_variables.target_velocity},
      {EP3ESLAVEPOS, MAXSINE, TARGET_POSITION, 0, &motor->drive_variables.target_postion},
      {EP3ESLAVEPOS, MAXSINE, STATUS_WORD, 0, &motor->drive_variables.status_word},
      {EP3ESLAVEPOS, MAXSINE, MODE_DISPLAY, 0, &motor->drive_variables.mode_display},
      {EP3ESLAVEPOS, MAXSINE, CURRENT_VELOCITY, 0, &motor->drive_variables.current_velocity},
      {EP3ESLAVEPOS, MAXSINE, ACUTAL_POSITION, 0, &motor->drive_variables.current_postion},
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
                                            {ACUTAL_POSITION, 0x00, 32}};

  // ec_pdo_info_t 用于初始化RxPDO以及TxPDO的映射表信息，结构为{RxPDO或TxPDO的对象字典索引，映射个数，ec_pdo_entry_info_t中RxPDO或TxPDO映射信息对应起始位}
  ec_pdo_info_t EP3E_pdos[] = {// RxPdo
                               {RXPDO, 4, EP3E_pdo_entries + 0},
                               // TxPdo
                               {TXPDO, 4, EP3E_pdo_entries + 4}};

  // ec_sync_info_t {通道编号，工作模式，通信目标数？，RxPDO或TxPDO映射表索引位置，看门狗模式？}
  ec_sync_info_t EP3E_syncs[] = {{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                                 {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                                 {2, EC_DIR_OUTPUT, 1, EP3E_pdos + 0, EC_WD_DISABLE},
                                 {3, EC_DIR_INPUT, 1, EP3E_pdos + 1, EC_WD_DISABLE},
                                 {0xFF}};

  // 创建ethercat主站master
  motor->master = ecrt_request_master(0);
  if (!motor->master)
  {
    printf("Failed to create ethercat master!\n");
    exit(EXIT_FAILURE); // 创建失败，退出线程
  }

  // 创建域domain
  motor->domain = ecrt_master_create_domain(motor->master);
  if (!motor->domain)
  {
    printf("Failed to create master domain!\n");
    exit(EXIT_FAILURE); // 创建失败，退出线程
  }

  // 配置从站
  if (!(motor->maxsine_EP3E = ecrt_master_slave_config(motor->master, EP3ESLAVEPOS, MAXSINE)))
  {
    printf("Failed to get slave configuration for EP3E!\n");
    exit(EXIT_FAILURE); // 配置失败，退出线程
  }

  // 对从站进行配置PDOs
  printf("Configuring PDOs...\n");
  if (ecrt_slave_config_pdos(motor->maxsine_EP3E, EC_END, EP3E_syncs))
  {
    printf("Failed to configure EP3E PDOs!\n");
    exit(EXIT_FAILURE); // 配置失败，退出线程
  }
  else
  {
    printf("*Success to configuring EP3E PDOs*\n"); // 配置成功
  }

  // 注册PDO entry
  if (ecrt_domain_reg_pdo_entry_list(motor->domain, domain_regs))
  {
    printf("PDO entry registration failed!\n");
    exit(EXIT_FAILURE); // 注册失败，退出线程
  }
  else
  {
    printf("*Success to configuring EP3E PDO entry*\n"); // 注册成功
  }

  // 激活主站master
  printf("Activating master...\n");
  if (ecrt_master_activate(motor->master))
  {
    exit(EXIT_FAILURE); // 激活失败，退出线程
  }
  else
  {
    printf("*Master activated*\n"); // 激活成功
  }
  if (!(motor->domain_pd = ecrt_domain_data(motor->domain)))
  {
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char **argv)
{
  struct MOTOR motor;
  init_EtherCAT_master(&motor);
  printf("*It's working now*\n");
  motor.targetPosition = 0;
  motor.opModeSet = 8;        // 位置模式
  int8_t reset_count = 0;     // 复位的时候计数用
  int16_t position_count = 0; // 位置模式下计数
  double t = 0.00;

  while (true)
  {
    usleep(250);

    // 接收过程数据
    ecrt_master_receive(motor.master); // EtherCAT 主站接收一次报文
    ecrt_domain_process(motor.domain); // 可以理解成根据报文更新一次domain域中的信息

    // 检查过程数据状态（可选）
    check_domain_state(motor.domain, &motor.domain_state);
    // 检查主站状态
    check_master_state(motor.master, &motor.master_state);
    // 检查从站配置状态
    check_slave_config_states(motor.maxsine_EP3E, &motor.maxsine_EP3E_state);

    // 读取数据，根据注册获得的PDO对象字典的偏移量进行读取
    motor.status =
        EC_READ_U16(motor.domain_pd + motor.drive_variables.status_word); // 状态字
    motor.opmode =
        EC_READ_U8(motor.domain_pd + motor.drive_variables.mode_display); // 运行模式
    motor.currentVelocity =
        EC_READ_S32(motor.domain_pd + motor.drive_variables.current_velocity); // 当前速度
    motor.currentPosition =
        EC_READ_S32(motor.domain_pd + motor.drive_variables.current_postion); // 当前位置
    motor.targetPosition =
        EC_READ_S32(motor.domain_pd + motor.drive_variables.target_postion); // 目标位置

    // DS402 CANOpen over EtherCAT status machine
    // 获取当前驱动器的驱动状态XZ
    if ((motor.status & 0x004F) == 0x0000)
      motor.driveState = dsNotReadyToSwitchOn; // 初始化 未完成状态
    else if ((motor.status & 0x004F) == 0x0040)
      motor.driveState = dsSwitchOnDisabled; //  初始化 完成状态 等待操作
    else if ((motor.status & 0x006F) == 0x0021)
      motor.driveState = dsReadyToSwitchOn; //  主电路电源OFF状态
    else if ((motor.status & 0x006F) == 0x0023)
      motor.driveState = dsSwitchedOn; // 伺服OFF/伺服准备
    else if ((motor.status & 0x006F) == 0x0027)
      motor.driveState = dsOperationEnabled; // 伺服ON
    else if ((motor.status & 0x006F) == 0x0007)
      motor.driveState = dsQuickStopActive; // 即停止
    else if ((motor.status & 0x004F) == 0x000F)
      motor.driveState = dsFaultReactionActive; // 异常（报警）判断
    else if ((motor.status & 0x004F) == 0x0008)
      motor.driveState = dsFault; // 异常（报警）状态

    printf("EP3E:  act position = %d , act velocity = %d , status = 0x%x , "
           "drivestate = %d , opmode = 0x%x\n",
           motor.currentPosition, motor.currentVelocity, motor.status,
           motor.driveState, motor.opmode);

    // 轮询检测是否在进行复位、急停或者使能
    // 复位
    if (motor.resetBusy == true)
    {
      reset_count = reset_count + 1;
      if (reset_count == 1)
      {
        EC_WRITE_U16(motor.domain_pd + motor.drive_variables.ctrl_word, 0x0);
      }
      else
      {
        EC_WRITE_U16(motor.domain_pd + motor.drive_variables.ctrl_word, 0x0080);
        if (motor.driveState == dsSwitchOnDisabled)
        {
          motor.resetBusy = false;
          reset_count = 0;
        }
      }
    }
    // 急停
    else if (motor.quickStopBusy == true)
    {
      // 控制驱动器的状态转化到Switch On Disabled
      switch (motor.driveState)
      {
        case dsSwitchedOn:
        case dsReadyToSwitchOn:
        case dsOperationEnabled:
          EC_WRITE_U16(motor.domain_pd + motor.drive_variables.ctrl_word, 0x0); // Disable Voltage
          break;

        default:
          motor.quickStopBusy = false;
          motor.positionMoving = false;
      }
    }
    // 使能
    else if (motor.powerBusy == true)
    {
      switch (motor.driveState)
      {
      case dsNotReadyToSwitchOn:
        break;

      case dsSwitchOnDisabled:
        // 设置运行模式为位置模式
        EC_WRITE_S8(motor.domain_pd + motor.drive_variables.operation_mode,
                    motor.opModeSet);
        EC_WRITE_U16(motor.domain_pd + motor.drive_variables.ctrl_word,
                     0x0006);
        break;

      case dsReadyToSwitchOn:
        EC_WRITE_U16(motor.domain_pd + motor.drive_variables.ctrl_word,
                     0x0007);
        break;

      case dsSwitchedOn:
        EC_WRITE_U16(motor.domain_pd + motor.drive_variables.ctrl_word,
                     0x000f); // enable operation
        motor.targetPosition = motor.currentPosition; // 将当前位置复制给目标位置，防止使能后电机震动
        EC_WRITE_S32(motor.domain_pd + motor.drive_variables.target_postion,
                     motor.targetPosition);
        break;

      default:
        motor.powerBusy = false;
      }
    }

    if (motor.driveState == dsOperationEnabled && motor.resetBusy == 0 &&
        motor.powerBusy == 0 && motor.quickStopBusy == 0)
        {
          if (motor.opmode == 8)
          { // 位置模式

            motor.targetPosition =
                motor.currentPosition + 500000;
            EC_WRITE_S32(motor.domain_pd + motor.drive_variables.target_postion,
                        motor.targetPosition);

            if (motor.homeBusy == true)
            { // 开始回零
              if (motor.currentPosition > 0)
              {
                motor.targetPosition = motor.currentPosition - HOME_STEP;
                if (motor.targetPosition < 0)
                {
                  motor.targetPosition = 0;
                }
              }
              else if (motor.currentPosition < 0)
              {
                motor.targetPosition = motor.currentPosition + HOME_STEP;
                if (motor.targetPosition > 0)
                {
                  motor.targetPosition = 0;
                }
              }
              else if (motor.currentPosition == 0)
              {
                motor.homeBusy = false; // 回零结束
                motor.positionMoving = false;
                t = 0;
              }
              EC_WRITE_S32(motor.domain_pd + motor.drive_variables.target_postion,
                          motor.targetPosition);
            }
            else
            {
              if (motor.positionMoving == true)
              { // 开始运动
                if (position_count == 8000)
                {
                  position_count = 0;
                  t = 0;
                }
                motor.targetPosition =
                    (int32_t)(ENCODER_RESOLUTION * sin(Pi * t) / 2); // 位置模式时传送位置信息
                // printf("电机实时位置%d\t,电机目标位置%d\n",motor.currentPosition,motor.targetPosition);
                EC_WRITE_S32(motor.domain_pd + motor.drive_variables.target_postion, motor.targetPosition);
                t = t + (double)POSITION_STEP;
                position_count = position_count + 1;

                EC_WRITE_S32(motor.domain_pd + motor.drive_variables.target_postion, motor.targetPosition);
              }
              else
              {
                motor.targetPosition = motor.currentPosition;
                EC_WRITE_S32(motor.domain_pd +
                                motor.drive_variables.target_postion,
                            motor.targetPosition);
              }
            }
          }
          EC_WRITE_U16(motor.domain_pd + motor.drive_variables.ctrl_word, 0x001f);
    }
    ecrt_domain_queue(motor.domain); // 根据domain数据生成帧
    ecrt_master_send(motor.master);  // EtherCAT主站发送
  }
  ecrt_master_deactivate(motor.master);
}