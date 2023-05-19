#!/bin/bash

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'
BOLD='\033[1m'

script=$(readlink -f "$0")
route=$(dirname "$script")
pkg_name=robot_hardware

echo -e "${GREEN}${BOLD}--- ${pkg_name} build begin ----${NC}"

## 1 is ready?
which cmake > /dev/null 2>&1 || { echo -e "${RED}cmake no found ${NC}"; exit 1; }

## 2 make build dir
echo -e "${BLUE}--- ${pkg_name} >>> create build dir begin ---"${NC}
if [ -d "${route}/../build" ]; then
    rm -rf ${route}/../build
fi
mkdir -p ${route}/../build || { echo -e "mkdir build failed"; exit 2; }
echo -e "${BLUE}--- ${pkg_name} >>> create build dir end ---"${NC}

## 3 make build
echo -e "${BLUE}--- ${pkg_name} >>> make build begin ---${NC}"
cd ${route}/../build
cpu_number=2
cpu_number=$(cat /proc/cpuinfo | grep processor | wc -l)
cmake .. -G "Unix Makefiles"
make -j ${cpu_number} || { echo -e "${RED}make fail${NC}"; exit 3; }
echo -e "${BLUE}--- ${pkg_name} >>> make build end ---${NC}"

echo -e "${GREEN}${BOLD}--- ${pkg_name} build end ----${NC}"

exit 0
