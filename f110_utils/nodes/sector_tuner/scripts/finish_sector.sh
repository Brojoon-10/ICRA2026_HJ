#!/bin/bash

source ~/.bashrc

cd ~/catkin_ws

### HJ : remove generated Config to force regeneration (catkin clean fails on merged develspace)
rm -f devel/lib/python3/dist-packages/sector_tuner/cfg/dyn_sect_tunerConfig.py
rm -f devel/lib/python3/dist-packages/sector_tuner/cfg/__pycache__/dyn_sect_tunerConfig*.pyc
rm -f devel/include/sector_tuner/dyn_sect_tunerConfig.h

catkin build sector_tuner
