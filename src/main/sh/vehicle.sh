#!/bin/sh
cd ${HOME}/Documents/Projects/Firmware || exit
cd Tools/jMAVSim/out/production || exit

hz=1

/usr/bin/java \
  -XX:GCTimeRatio=20 \
  -Djava.ext.dirs= \
  -Djavax.accessibility.assistive_technologies= \
  -jar jmavsim_run.jar \
  -tcp 127.0.0.1:4560 \
  -r ${hz} \
  -lockstep
