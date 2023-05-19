#!/bin/bash

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'
BOLD='\033[1m'

echo -e "${GREEN}${BOLD}--- robot_hardware runtime begin ----${NC}"

## 1.0 environment variable
script=$(readlink -f "$0")
route=$(dirname "$script")
lib_server="ftp://test:test@192.168.1.100/soferware/lib"
deb_pkgs=()
log_dir=/opt/comwise/log

if [ ! -d ${log_dir} ]; then
    sudo mkdir -p ${log_dir}
fi
session_user=`echo ${SUDO_USER:-$USER}`
uid=`id -un ${session_user}`
gid=`id -gn ${session_user}`
sudo chown -R ${uid}:${gid} ${log_dir}
sudo touch ${log_dir}/robot_hardware.log
sudo chown ${uid}:${gid} ${log_dir}/robot_hardware.log
sudo chmod 664 ${log_dir}/robot_hardware.log

## 2. install lib
apt_pkgs+=(
    # log
    libspdlog-dev

    #jsoncpp
    libjsoncpp-dev

    # process
    libpoco-dev
    libtinyxml-dev

    # net_utils
    libcap-dev
    libcap2-bin
    net-tools

    # pack app tool
    fakeroot

    # run app tools
    circus
)

for n in ${apt_pkgs[@]}
do
    echo -e "${RED}Processing dpkg pkg ----> " $n ${NC}
    sudo apt-get install -y $n || { fail_dpkgs+=($n); continue; }
    echo -e "${RED}Processing dpkg pkg ----> " $n " $BOLD OK" ${NC}
done

## 3 install deb
deb_pkgs+=(

)
mkdir -p ~/.tmp
tmp_dir=~/.tmp
for n in ${deb_pkgs[@]}
do
    echo -e "${RED}Processing deb ----> " $n ${NC}
    [ -f "${tmp_dir}/${n}" ] || { wget -P ${tmp_dir} ${lib_server}/${n} || { fail_debs+=($n); continue; } }
    sudo dpkg -E -i ${tmp_dir}/${n} || { echo -e "${RED}Fail installing debs: $BOLD ${fail_debs[@]}" ${NC}; continue; }
    echo -e "${RED}Processing deb----> " $n "$BOLD OK" ${NC}
done
rm -rf ${tmp_dir}

echo -e "${GREEN}${BOLD}--- robot_hardware runtime end ----${NC}"

exit 0
