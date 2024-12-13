#ifndef _ETHERCAT_GLOBAL_H_
#define _ETHERCAT_GLOBAL_H_

#include <ecrt.h>
#include <stdio.h>
#include <time.h>
#include <stdint.h>

#define NSEC_PER_SEC 1000000000
#define Pi 3.141592654                                               // 圆周率

#define DEBUG

// ethercat主站应用程序所用的参数
#define ENABLE_DC
#define ENABLE_SDO

#define TASK_FREQUENCY 1000                                          // Hz 任务周期（任务频率）
#define POSITION_STEP (1 / TASK_FREQUENCY)                           // 位置模式下步长
#define TASK_PERIOD_NS (NSEC_PER_SEC / TASK_FREQUENCY)
#define SHIFT_NS (TASK_PERIOD_NS / 4)
#define PRINT_COUNT (TASK_FREQUENCY / 50)                            // 固定0.5s输出一次
// #define UPDATE_COUNT (TASK_FREQUENCY / 100)                          // 固定10ms更新一次控制数据，在CSP模式下建议每个通讯周期都更新位置，避免加速度和速度波动导致急停，建议使用0x6091关键字设置电子齿轮

#define MOTOR_ENCODER_RESOLUTION 8388608                              // 编码器分辨率 2^23
#define MOTOR_RATED_TORQUE 0.64                                       // 电机额定转矩0.64N/m
#define SCREW_MOVE_VELOCITY 10                                        // mm/s，丝杠运动速度定义
#define SCREW_MINUS_MOVE_UNIT 0.02                                    // mm，丝杠最小运动单位
#define HOME_VECOLITY 20                                               // mm/s，丝杠回零速度
#define HOME_STEP (HOME_VECOLITY / SCREW_MINUS_MOVE_UNIT / TASK_FREQUENCY)  // pulse 回零步长 = 丝杠回零速度 / 丝杠最小运动单位 / 任务频率
#define MOVE_DIRECTION 1                                              // 电机旋转方向定义


#define ETHERCAT_INFO(format, ...) \
          do{\
              printf("\033[0m [EtherCAT Info] " format "\n", ##__VA_ARGS__);\
            }while (false)
#define ETHERCAT_MESSAGE(format, ...) \
          do{\
            printf("\033[0;32;40m [EtherCAT Info] " format "\033[0m\n", ##__VA_ARGS__);\
            }while (false)
#define ETHERCAT_WARNING(format, ...) \
          do{\
              printf("\033[0;33;40m [EtherCAT Warning] " format "\033[0m\n", ##__VA_ARGS__);\
            }while (false)
#define ETHERCAT_ERROR(format, ...) \
          do{\
              printf("\033[0;31;40m [EtherCAT Error] " format "\033[0m\n", ##__VA_ARGS__);\
              exit(EXIT_FAILURE);\
            }while(false)

#define TIMESPEC2NS(T) ((uint64_t) ((T).tv_sec * NSEC_PER_SEC + (T).tv_nsec))
#define DIFF_NS_TIMESPEC(T1, T2) ((uint64_t) (TIMESPEC2NS(T1) - TIMESPEC2NS(T2)))


const struct timespec timespec_add(const struct timespec& time1, const struct timespec& time2);
const struct timespec timespec_minus(const struct timespec& time1, const struct timespec& time2);

#endif