#ifndef _ETHERCAT_GLOBAL_H_
#define _ETHERCAT_GLOBAL_H_

#include <ecrt.h>
#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <signal.h>
#include <pthread.h>

#define ETHERCAT_DEBUG_MODE

// ethercat主站应用程序所用的参数
// #define ENABLE_DC
#define ENABLE_SDO

constexpr int NSEC_PER_SEC = 1000000000;
constexpr double Pi = 3.141592654;                                   // 圆周率

constexpr int TASK_FREQUENCY = 1000;                                // Hz 任务周期（任务频率）
// constexpr double POSITION_STEP = 1 / TASK_FREQUENCY;                // 位置模式下步长
constexpr int TASK_PERIOD_NS = NSEC_PER_SEC / TASK_FREQUENCY;
constexpr int SHIFT_NS = TASK_PERIOD_NS / 4;
constexpr int PRINT_COUNT = TASK_FREQUENCY / 2;                    // 固定0.5s输出一次
constexpr int UPDATE_COUNT = TASK_FREQUENCY / 100;                  // 固定10ms更新一次控制数据，建议CSP模式不要使用，会导致加速度和速度波动，可能导致自动急停

constexpr int MOTOR_ENCODER_RESOLUTION = 8388608;                   // 编码器分辨率 2^23
constexpr double MOTOR_RATED_TORQUE = 0.64;                         // 电机额定转矩0.64N/m
constexpr double SCREW_MOVE_VELOCITY = 2;                           // mm/s，丝杠运动速度定义
constexpr double SCREW_MINUS_MOVE_UNIT = 0.02;                      // mm，丝杠最小运动单位
constexpr double HOME_VECOLITY = 20;                                // mm/s，丝杠回零速度
constexpr double HOME_STEP = 20;  // pulse 回零步长 = 丝杠回零速度 / 丝杠最小运动单位 / 任务频率
constexpr bool MOVE_DIRECTION = false;                               // 电机旋转方向定义
// constexpr bool MOVE_DIRECTION = true;                               // 电机旋转方向定义
constexpr int32_t MOVE_VELOCITY = 1;                               // 电机旋转速度定义


#define ETHERCAT_INFO(format, ...) \
          do{\
              printf("\033[0m [EtherCAT INFO] " format "\n", ##__VA_ARGS__);\
            }while (false)
#define ETHERCAT_DEBUG(format, ...) \
          do{\
              printf("\033[0;36m [EtherCAT DEBUG] " format "\n", ##__VA_ARGS__);\
            }while (false)
#define ETHERCAT_MESSAGE(format, ...) \
          do{\
            printf("\033[0;32m [EtherCAT MESSAGE] " format "\033[0m\n", ##__VA_ARGS__);\
            }while (false)
#define ETHERCAT_WARNING(format, ...) \
          do{\
              printf("\033[0;33m [EtherCAT WARNING] " format "\033[0m\n", ##__VA_ARGS__);\
            }while (false)
#define ETHERCAT_CRITICAL(format, ...) \
          do{\
              printf("\033[0;35m [EtherCAT CRITIAL] " format "\033[0m\n", ##__VA_ARGS__);\
            }while(false)
#define ETHERCAT_ERROR(format, ...) \
          do{\
              printf("\033[0;31m [EtherCAT ERROR] " format "\033[0m\n", ##__VA_ARGS__);\
              exit(EXIT_FAILURE);\
            }while(false)

#define TIMESPEC2NS(T) ((uint64_t) ((T).tv_sec * NSEC_PER_SEC + (T).tv_nsec))
#define DIFF_NS_TIMESPEC(T1, T2) ((uint64_t) (TIMESPEC2NS(T1) - TIMESPEC2NS(T2)))


const struct timespec timespec_add(const struct timespec& time1, const struct timespec& time2);
const struct timespec timespec_minus(const struct timespec& time1, const struct timespec& time2);

#endif