#include "srv/hardware_manage.h"
#include <thread>
#include <math.h>

int main (int argc, char** argv)
{
    comwise::srv::hardware_manage mgr;
    
    mgr.start();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    mgr.stop();

    return 0;
}
