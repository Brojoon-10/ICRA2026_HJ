#!/bin/bash

source ~/.bashrc

cd ~/catkin_ws

### HJ : remove generated Config to force regeneration (catkin clean fails on merged develspace)
rm -f devel/lib/python3/dist-packages/sector_tuner_3d/cfg/dyn_sect_tunerConfig.py
rm -f devel/lib/python3/dist-packages/sector_tuner_3d/cfg/__pycache__/dyn_sect_tunerConfig*.pyc
rm -f devel/include/sector_tuner_3d/dyn_sect_tunerConfig.h

catkin build sector_tuner_3d
