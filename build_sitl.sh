#!/bin/bash

./waf configure --debug --board sitl
./waf copter


if [[ "$OSTYPE" == "darwin"* ]]; then
  cp build/sitl/bin/arducopter ../skybrush-dev-setup/ap-swarm-launcher/sitl_local/arducopter
  echo "Running on Mac"
else
  cp build/sitl/bin/arducopter ../skybrush-dev-setup/ap-swarm-launcher/sitl_local_ubuntu/arducopter
  echo "Running on Ubuntu"
fi


