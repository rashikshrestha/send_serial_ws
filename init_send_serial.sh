#!/bin/bash

echo "#----------------- THIS IS TO SOURCE send_serial__ws PACKAGES -----------------" >> ~/.bashrc

echo "export SEND_SERIAL_WS=$(pwd)" >> ~/.bashrc
echo 'source $SEND_SERIAL_WS/devel/setup.bash' >> ~/.bashrc
echo 'source $SEND_SERIAL_WS/send_serial.sh' >> ~/.bashrc

echo "#---------------------------------------------------------------------" >> ~/.bashrc

source ~/.bashrc 
