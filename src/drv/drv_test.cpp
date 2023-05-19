
#include <thread>
#include <chrono>
#include "drv/drv_impl.h"

void sleep(uint32_t ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main()
{
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}