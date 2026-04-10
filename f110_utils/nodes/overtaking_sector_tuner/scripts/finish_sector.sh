#!/bin/bash

source ~/.bashrc

cd ~/catkin_ws

### HJ : remove generated Config to force regeneration (catkin clean fails on merged develspace)
rm -f devel/lib/python3/dist-packages/overtaking_sector_tuner/cfg/ot_dyn_sect_tunerConfig.py
rm -f devel/lib/python3/dist-packages/overtaking_sector_tuner/cfg/__pycache__/ot_dyn_sect_tunerConfig*.pyc
rm -f devel/include/overtaking_sector_tuner/ot_dyn_sect_tunerConfig.h

catkin build overtaking_sector_tuner
