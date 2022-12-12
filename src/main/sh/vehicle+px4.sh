#!/bin/sh
cd ${HOME}/Documents/Projects/Firmware || exit
export JAVA_HOME=/Library/Java/JavaVirtualMachines/jdk1.8.0_221.jdk/Contents/Home
make px4_sitl jmavsim