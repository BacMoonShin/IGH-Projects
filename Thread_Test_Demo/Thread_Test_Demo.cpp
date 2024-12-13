/*
 * @Author: your name
 * @Date: 2024-09-20 14:44:49
 * @LastEditors: your name
 * @LastEditTime: 2024-09-21 14:21:35
 * @Description:
 * @FilePath: /Code/Thread_Test_Demo.cpp
 */
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <unistd.h> // for sleep
#include <ctime>    // 引入时间函数
#include <iomanip>
#include <mutex>  // 引入线程锁
#include <chrono> // 引入高精度定时器

// 定义一个全局的互斥锁
std::mutex DataMutex;

// 定义共享数据结构
struct ShareData
{
    // 可以在此处添加想要共享的数据类型
    int count;
};

// 获取当前系统时间并输出
void printCurrentTime()
{
    std::time_t currentTime = std::time(nullptr);      // 获取当前时间
    std::tm *localTime = std::localtime(&currentTime); // 转换为本地时间

    // 格式化输出时间
    std::cout << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << std::endl;
}

// 定义线程1函数
void *threadFunction1(void *arg)
{
    ShareData *shareData = static_cast<ShareData *>(arg);  // 通过转换指针类型得到共享数据的指针
    const int period_ms = 3000;                            // 定义线程1的运行周期（毫秒）
    auto next_run_time = std::chrono::steady_clock::now(); // 获取第一次运行时间

    while (true)
    {
        next_run_time += std::chrono::milliseconds(period_ms); // 计算下一次运行时间
        auto start_time = std::chrono::steady_clock::now();    // 记录任务开始时间

        //***************** 自定义函数调用内容 *****************

        // 涉及访问和修改共享数据的函数部分要锁定和释放互斥锁以避免造成访问冲突

        DataMutex.lock(); // 锁定互斥锁
        std::cout << "Thread 1 is running at ";
        printCurrentTime(); // 输出当前系统时间
        shareData->count += 1;
        std::cout << "Thread 1 updated sharedata to: " << shareData->count << std::endl;
        DataMutex.unlock(); // 释放互斥锁

        //***************** 自定义函数调用内容 *****************

        auto end_time = std::chrono::steady_clock::now();                                                   // 记录任务结束时间
        auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time); // 计算任务运行时间

        // 检查任务运行时间是否超出设定的运行周期，超出则发出警告并输出超出的时间
        if (execution_time.count() > period_ms)
        {
            std::cout << "Warning: Thread 1 execution time exceeded the period by " << (execution_time.count() - period_ms) << " ms!" << std::endl;
            next_run_time = end_time; // 设置下一次线程函数立刻执行，更新下一次运行时间
        }
        else
        {
            auto sleep_time = std::chrono::duration_cast<std::chrono::milliseconds>(next_run_time - std::chrono::steady_clock::now()); // 计算当前距离下一次运行的休眠时间
            if (sleep_time.count() > 0)
            {
                usleep(sleep_time.count() * 1000); // usleep采用微秒计时
            }
        }
    }
    return nullptr;
}

// 定义线程2函数
void *threadFunction2(void *arg)
{
    ShareData *shareData = static_cast<ShareData *>(arg);  // 通过转换指针类型得到共享数据的指针
    const int period_ms = 2000;                            // 定义线程2的运行周期（毫秒）
    auto next_run_time = std::chrono::steady_clock::now(); // 获取第一次运行时间

    while (true)
    {
        next_run_time += std::chrono::milliseconds(period_ms); // 计算下一次运行时间
        auto start_time = std::chrono::steady_clock::now();    // 记录任务开始时间

        //***************** 自定义函数调用内容 *****************

        // 涉及访问和修改共享数据的函数部分要锁定和释放互斥锁以避免造成访问冲突
        DataMutex.lock(); // 锁定互斥锁
        std::cout << "Thread 2 is running at ";
        printCurrentTime(); // 输出当前系统时间
        shareData->count += 2;
        std::cout << "Thread 2 updated sharedata to: " << shareData->count << std::endl;
        DataMutex.unlock(); // 释放互斥锁

        //***************** 自定义函数调用内容 *****************

        auto end_time = std::chrono::steady_clock::now();                                                   // 记录任务结束时间
        auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time); // 计算任务运行时间

        // 检查任务运行时间是否超出设定的运行周期，超出则发出警告并输出超出的时间
        if (execution_time.count() > period_ms)
        {
            std::cout << "Warning: Thread 1 execution time exceeded the period by " << (execution_time.count() - period_ms) << " ms!" << std::endl;
            next_run_time = end_time; // 设置下一次线程函数立刻执行，更新下一次运行时间
        }
        else
        {
            auto sleep_time = std::chrono::duration_cast<std::chrono::milliseconds>(next_run_time - std::chrono::steady_clock::now()); // 计算当前距离下一次运行的休眠时间
            if (sleep_time.count() > 0)
            {
                usleep(sleep_time.count() * 1000); // usleep采用微秒计时
            }
        }
    }
    return nullptr;
}

// 定义线程3函数
void *threadFunction3(void *arg)
{
    ShareData *shareData = static_cast<ShareData *>(arg);  // 通过转换指针类型得到共享数据的指针
    const int period_ms = 1500;                            // 定义线程3的运行周期（毫秒）
    auto next_run_time = std::chrono::steady_clock::now(); // 获取第一次运行时间

    while (true)
    {
        next_run_time += std::chrono::milliseconds(period_ms); // 计算下一次运行时间
        auto start_time = std::chrono::steady_clock::now();    // 记录任务开始时间

        //***************** 自定义函数调用内容 *****************

        // 涉及访问和修改共享数据的函数部分要锁定和释放互斥锁以避免造成访问冲突
        DataMutex.lock(); // 锁定互斥锁
        std::cout << "Thread 3 is running at ";
        printCurrentTime(); // 输出当前系统时间
        shareData->count += 3;
        std::cout << "Thread 3 updated sharedata to: " << shareData->count << std::endl;
        DataMutex.unlock(); // 释放互斥锁

        //***************** 自定义函数调用内容 *****************

        auto end_time = std::chrono::steady_clock::now();                                                   // 记录任务结束时间
        auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time); // 计算任务运行时间

        // 检查任务运行时间是否超出设定的运行周期，超出则发出警告并输出超出的时间
        if (execution_time.count() > period_ms)
        {
            std::cout << "Warning: Thread 3 execution time exceeded the period by " << (execution_time.count() - period_ms) << " ms!" << std::endl;
            next_run_time = end_time; // 设置下一次线程函数立刻执行，更新下一次运行时间
        }
        else
        {
            auto sleep_time = std::chrono::duration_cast<std::chrono::milliseconds>(next_run_time - std::chrono::steady_clock::now()); // 计算当前距离下一次运行的休眠时间
            if (sleep_time.count() > 0)
            {
                usleep(sleep_time.count() * 1000); // usleep采用微秒计时
            }
        }
    }
    return nullptr;
}

int main()
{
    // 开辟用于共享数据的内存
    ShareData *shareData = new ShareData;

    pthread_t thread1, thread2, thread3;
    cpu_set_t cpuset;
    struct sched_param param1, param2, param3;

    // 创建线程1
    pthread_create(&thread1, nullptr, threadFunction1, shareData);

    // 设置线程1的优先级
    param1.sched_priority = 10;                        // 设置线程1优先级为10
    pthread_setschedparam(thread1, SCHED_RR, &param1); // pthread_setschedparam函数针对线程优先级进行设置，SCHED_RR表示将根据优先级使用轮转的实时调度策略，允许同优先级的线程进行轮转运行

    // 设置线程1的CPU亲和性（绑定到CPU 0）
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset); // 设置线程1只能在CPU 0上运行
    pthread_setaffinity_np(thread1, sizeof(cpu_set_t), &cpuset);

    // 创建线程2
    pthread_create(&thread2, nullptr, threadFunction2, shareData);

    // 设置线程2的优先级
    param2.sched_priority = 20; // 设置线程2优先级为20
    pthread_setschedparam(thread2, SCHED_RR, &param2);

    // 设置线程2的CPU亲和性（绑定到CPU 1）
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset); // 设置线程2只能在CPU 1上运行
    pthread_setaffinity_np(thread2, sizeof(cpu_set_t), &cpuset);

    // 创建线程3
    pthread_create(&thread3, nullptr, threadFunction3, shareData);

    // 设置线程3的优先级
    param2.sched_priority = 20; // 设置线程3优先级为20
    pthread_setschedparam(thread3, SCHED_RR, &param3);

    // 设置线程3的CPU亲和性（绑定到CPU 1）
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset); // 设置线程3只能在CPU 2上运行
    pthread_setaffinity_np(thread3, sizeof(cpu_set_t), &cpuset);

    // 等待线程结束
    pthread_join(thread1, nullptr);
    pthread_join(thread2, nullptr);
    pthread_join(thread3, nullptr);

    delete shareData;

    return 0;
}