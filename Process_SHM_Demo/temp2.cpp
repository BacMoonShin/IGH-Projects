#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

using namespace std;

// 共享内存：让父子进程共享内存
int main()
{

    int fd = open("/dev/zero", O_RDWR); // /dev/zero创建共享内存的一个特殊文件
    if (-1 == fd)
    { // 判断文件是否打开成功
        perror("open error");
        return 1;
    }
    void *buff = mmap(NULL, sizeof(int), PROT_WRITE | PROT_READ, MAP_SHARED, fd, 0);
    if (MAP_FAILED == buff)
    { // 判断文件是否映射成功
        perror("mmap error");
        return 1;
    }
    int &n = *static_cast<int *>(buff);
    if (fork() == 0)
    {
        for (int i = 0; i < 10; i++)
            cout << getpid() << ":" << ++n << endl;
    }
    else
    {
        for (int i = 0; i < 10; i++)
            cout << getpid() << ":" << --n << endl;
    }

    munmap(buff, sizeof(int));
    buff = NULL;
    close(fd);
}
