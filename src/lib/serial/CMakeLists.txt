cmake_minimum_required(VERSION 3.2)

add_library(serial_sdk SHARED
  serial.cc
  impl/unix.cc
  impl/win.cc
  impl/list_ports/list_ports_win.cc
  impl/list_ports/list_ports_linux.cc
  impl/list_ports/list_ports_osx.cc
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)