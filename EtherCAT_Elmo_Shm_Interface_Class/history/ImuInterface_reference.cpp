#include <imu_communication/ImuCommunication.hpp>
#include <imu_communication/ImuInterface.hpp>

#include <SharedMemory.hpp>
#include <SharedMemoryDataType.hpp>

// 200Hz更新的Imu数据
std::vector<float> imuValues200Hz;
// Imu共享内存变量
SharedDataConfig sharedDataConfig;
SharedMemory *imuValuesSm;
ImuData imuValuesOfRead2Sm;
ImuData imuValuesOfWrite2Sm;
// 定义定时器频率
const int TIMER_FREQ_HZ = 200; // 200 Hz
// 定义周期时间（纳秒）
const long long PERIOD_NS = 1000000000LL / TIMER_FREQ_HZ;

int main(int argc, char** argv)
{
    // 创建Imu共享内存方法
    try{
        imuValuesSm = new SharedMemory(sharedDataConfig.FILE_PATH, sharedDataConfig.length[6], sharedDataConfig.keyNum[6], 0, sharedDataConfig.semName[6]);
        imuValuesSm->connect();
    }catch(const std::runtime_error&e)
    {
        std::cerr << "Caught an error: " << e.what() << '\n';
        return 0; 
    };
    // Imu变量初始化
    imuValues200Hz.resize(IMU_DATA_DIMENSION);
    // 创建人形机器人Imu调度定时器
    // struct timespec imuTimer;
    // imuTimer.tv_sec = 0;
    // imuTimer.tv_nsec = 1 * 1000 * 1000; // 1ms
    struct timespec ts;
    auto next_wakeup = std::chrono::high_resolution_clock::now();
    float lastTime = 0;
    float durationTime = 0;
    // 创建人形机器人Imu类
    Imu humanoidImuObject(IMU_SERIAL_PORT, IMU_BAUDRATE); 
    // 打开人形机器人Imu串口
    uint8_t serialOpenState = humanoidImuObject.openImuSerialPort();
    if(serialOpenState != 1) return 0;
    // Imu设置初始化，执行一次
    else
    {
        humanoidImuObject.imuWriteUnlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        humanoidImuObject.imuSettingTransmittingSpeed();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        humanoidImuObject.imuSettingSave();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        humanoidImuObject.imuWriteUnlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        humanoidImuObject.imuSettingTransmittingValues();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        humanoidImuObject.imuSettingSave();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        humanoidImuObject.imuWriteUnlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        humanoidImuObject.imuSettingSamplingBandwidth();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        humanoidImuObject.imuSettingSave();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        humanoidImuObject.imuStartReadingData();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "IMU初始化完成, 开始读取数据" << std::endl; 
    }
    // 主循环调度，以1000Hz的频率更新IMU数据并写入共享内存中
    // 获取循环开始时间
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while(true)
    {
        // 获取当前时间
        auto start_time = std::chrono::high_resolution_clock::now();
        clock_gettime(CLOCK_MONOTONIC, &ts);
        std::cout << "Timer tick at " << ts.tv_sec << "s " << ts.tv_nsec << "ns" << std::endl;
        durationTime = ts.tv_nsec - lastTime;
        std::cout << "运行一次的总时间为："<< durationTime/(1000 * 1000) << "ms" <<std::endl;
        lastTime = ts.tv_nsec;
        // 获取Imu的数据
        imuValues200Hz = humanoidImuObject.imuValuesUpdate();
        // Imu数据更新到代写入共享内存的变量中
        imuValuesSm->write(&imuValues200Hz[0], IMU_DATA_DIMENSION * sizeof(float), 0);
        // 测试共享内存是否写成功
        // imuValuesSm->read(&imuValuesOfRead2Sm, 1, 0);
        std::cout << "IMU加速度传感器数据x:" << imuValues200Hz[0] << ";" << "IMU加速度传感器数据y:" << imuValues200Hz[1] << ";" << "IMU加速度传感器数据z:" << imuValues200Hz[2] << std::endl;
        std::cout << "IMU角速度传感器数据x:" << imuValues200Hz[3] << ";" << "IMU角速度传感器数据y:" << imuValues200Hz[4] << ";" << "IMU角速度传感器数据z:" << imuValues200Hz[5] << std::endl;
        std::cout << "IMU角度传感器数据x:" << imuValues200Hz[6] << ";" << "IMU角度传感器数据y:" << imuValues200Hz[7] << ";" << "IMU角度传感器数据z:" << imuValues200Hz[8] << std::endl;
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::nano> elapsed_time = end_time - start_time;

        // 计算剩余的睡眠时间
        auto sleep_time = std::chrono::nanoseconds(PERIOD_NS) - elapsed_time;
        if (sleep_time > std::chrono::nanoseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        }

        // 更新下一个唤醒时间
        next_wakeup += std::chrono::nanoseconds(PERIOD_NS);
    }
    return 0;
}