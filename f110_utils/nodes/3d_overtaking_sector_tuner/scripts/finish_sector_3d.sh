#!/bin/bash

source ~/.bashrc

cd ~/catkin_ws

### HJ : remove generated Config to force regeneration (catkin clean fails on merged develspace)
# origin: rm -f devel/lib/python3/dist-packages/overtaking_sector_tuner_3d/cfg/ot_dyn_sect_tunerConfig.py
# origin: rm -f devel/lib/python3/dist-packages/overtaking_sector_tuner_3d/cfg/__pycache__/ot_dyn_sect_tunerConfig*.pyc
# origin: rm -f devel/include/overtaking_sector_tuner_3d/ot_dyn_sect_tunerConfig.h
## IY : new self-contained 3D Config name
rm -f devel/lib/python3/dist-packages/overtaking_sector_tuner_3d/cfg/ot_dyn_sect_tuner_3dConfig.py
rm -f devel/lib/python3/dist-packages/overtaking_sector_tuner_3d/cfg/__pycache__/ot_dyn_sect_tuner_3dConfig*.pyc
rm -f devel/include/overtaking_sector_tuner_3d/ot_dyn_sect_tuner_3dConfig.h

catkin build overtaking_sector_tuner_3d
