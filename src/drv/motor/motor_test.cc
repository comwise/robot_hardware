#include <thread>
#include <chrono>
#include <sstream>
#include <iostream>
#include "motor/motor_driver.h"

void data_cb(const comwise::motor::motor_data_t &data)
{
  std::stringstream ss;
  ss << data.time_stamp;
  ss << " speed:" << data.speed;
  ss << " voltage:" << data.voltage;
  ss << " current:" << data.current;
  
  std::cout << ss.str() << std::endl;
}

int main()
{
  comwise::motor::motor_driver drv("123");
  comwise::motor::motor_param_t param;
  drv.init(param);
  //drv.set_data_cb(data_cb);
  while(true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return 0;
}