#include "joy_stick.h"
#include <thread>
#include <chrono>
#include <sstream>
#include <iostream>

void joy_cb(const comwise::joy_data& data)
{
  std::stringstream ss;
  ss << data.time;
  ss << " button:";
  for(auto &item : data.buttons) {
    ss << " " << item;
  }
  ss << " axes:";
  for(auto &item : data.axes) {
    ss << " " << item;
  }
  std::cout << ss.str() << std::endl;
}

int main()
{
  comwise::joy::joy_stick joy("/dev/input/js0");
  joy.set_data_callback(joy_cb);
  joy.start();
  while(true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  joy.stop();
  return 0;
}