#!/bin/bash

sudo rm  -r data_log
mkdir data_log
bzzc battery_swap_cognifly.bzz
#./bzzCognifly tcp 500 *.bo *.bdb
./bzzCognifly tcp 500 *.bo *.bdb 2 192.168.0.113
