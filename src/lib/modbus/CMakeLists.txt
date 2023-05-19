cmake_minimum_required(VERSION 2.8.3)

add_library(modbus_sdk SHARED
  modbus-data.c
  modbus-tcp.c
  modbus-rtu.c
  modbus.c
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)