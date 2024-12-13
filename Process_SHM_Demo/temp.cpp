#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h> // For mode constants
#include <fcntl.h>    // For O_* constants
#include <unistd.h>
#include <cstring> // For memcpy
#include <semaphore.h>
#include <sys/wait.h>

#define SHM_NAME "/shm_name"
#define SEM_PARENT "/sem_parent"
#define SEM_CHILD "/sem_child"
#define SHM_SIZE 200  // 共享内存大小
#define ITERATIONS 10 // 通信的循环次数

int main()
{
    // 创建共享内存对象
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        std::cerr << "Failed to create shared memory" << std::endl;
        return 1;
    }

    // 调整共享内存大小
    if (ftruncate(shm_fd, SHM_SIZE) == -1)
    {
        std::cerr << "Failed to set size for shared memory" << std::endl;
        return 1;
    }

    // 映射共享内存到进程地址空间
    void *shared_memory = mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_memory == MAP_FAILED)
    {
        std::cerr << "Failed to map shared memory" << std::endl;
        return 1;
    }

    // 创建或打开信号量（用于同步）
    sem_t *sem_parent = sem_open(SEM_PARENT, O_CREAT, 0666, 1); // 父进程初始获得信号量
    sem_t *sem_child = sem_open(SEM_CHILD, O_CREAT, 0666, 0);   // 子进程初始阻塞

    if (sem_parent == SEM_FAILED || sem_child == SEM_FAILED)
    {
        std::cerr << "Failed to create semaphores" << std::endl;
        return 1;
    }

    // 创建子进程
    pid_t pid = fork();
    if (pid == -1)
    {
        std::cerr << "Fork failed" << std::endl;
        return 1;
    }

    if (pid == 0)
    { // 子进程
        for (int i = 0; i < ITERATIONS; ++i)
        {
            // 等待父进程完成写入
            sem_wait(sem_child);

            // 从共享内存读取数据
            char buffer[SHM_SIZE];
            memcpy(buffer, shared_memory, SHM_SIZE);
            std::cout << "Child process read: " << buffer << std::endl;

            // 写入子进程的消息到共享内存
            std::string child_message = "Message from child process, iteration " + std::to_string(i);
            memcpy(shared_memory, child_message.c_str(), child_message.size() + 1);

            // 释放信号量，通知父进程
            sem_post(sem_parent);
        }

        // 解除映射并关闭共享内存和信号量
        munmap(shared_memory, SHM_SIZE);
        sem_close(sem_parent);
        sem_close(sem_child);
    }
    else
    { // 父进程
        sem_wait(sem_parent);
        for (int i = 0; i < ITERATIONS; ++i)
        {
            // 写入父进程的消息到共享内存
            std::string parent_message = "Message from parent process, iteration " + std::to_string(i);
            memcpy(shared_memory, parent_message.c_str(), parent_message.size() + 1);

            // 释放信号量，通知子进程
            sem_post(sem_child);

            // 等待子进程读取并写入数据
            sem_wait(sem_parent);

            // 从共享内存读取子进程的数据
            char buffer[SHM_SIZE];
            memcpy(buffer, shared_memory, SHM_SIZE);
            std::cout << "Parent process read: " << buffer << std::endl;
        }

        // 等待子进程结束
        wait(&pid);

        // 解除映射并删除共享内存和信号量
        munmap(shared_memory, SHM_SIZE);
        shm_unlink(SHM_NAME);
        sem_close(sem_parent);
        sem_close(sem_child);
        sem_unlink(SEM_PARENT);
        sem_unlink(SEM_CHILD);
    }

    return 0;
}
