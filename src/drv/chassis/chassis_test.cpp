#include "chassis/chassis_impl.h"
#include <thread>

static const double PI = 3.1415926;

int main()
{
    int i = 0;
    comwise::chassis::chassis_impl impl("test");
    //impl.set_speed(PI/4, PI/4);
    impl.start();
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        i++;
        if(i == 82) {
            printf("count ok, i=%d\n", i);
            impl.stop();
        }
    }
    impl.stop();
    return 0;
}

