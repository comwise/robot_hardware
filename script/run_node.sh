#!/usr/bin/env bash

shutdown() {
  # Get our process group id
  PGID=$(ps -o pgid= $$ | grep -o [0-9]*)
  # kill it in a new new process group
  kill_node
  setsid kill -2 -- -${PGID}
  exit 0
}

kill_node() {
  ps -ef | grep robot_hardware | grep -v grep | cut -c 9-15 | xargs kill -9
}

trap 'shutdown' SIGINT SIGTERM

source /opt/ros/kinetic/setup.bash --extend|| { echo `date` ": source ros bash failed"; exit 1; }
source /opt/comwise/robot_hardware/devel/setup.bash --extend  || { echo `date` ": source robot_hardware bash failed"; exit 2; }
/opt/comwise/robot_hardware/bin/robot_hardware || { echo `date` ": run robot_hardware failed"; exit 3; }
wait
exit 0
