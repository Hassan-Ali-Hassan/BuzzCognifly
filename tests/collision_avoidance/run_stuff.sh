#!/bin/bash

sudo rm  -r data_log
mkdir data_log
bzzc collision_avoidance_mission.bzz
#./bzzCognifly tcp 500 *.bo *.bdb
./bzzCognifly tcp 500 *.bo *.bdb 2 192.168.0.102
