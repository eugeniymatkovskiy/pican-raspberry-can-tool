#!/bin/bash
while true; do
  cansend can0 7DF#0201040000000000 # 04: Engine load
  sleep 0.05
  cansend can0 7DF#0201050000000000 # 05: Antifreeze
  sleep 0.05
  cansend can0 7DF#02010B0000000000 # 0B: MAP (supercharging)
  sleep 0.05
  cansend can0 7DF#02010C0000000000 # 0C: RPM
  sleep 0.05
  cansend can0 7DF#02010D0000000000 # 0D: Speed
  sleep 0.05
  cansend can0 7DF#0201100000000000 # 10: Air consumption (MAF)
  sleep 0.05
  cansend can0 7DF#0201110000000000 # 11: Throttle
  sleep 0.05
  cansend can0 7DF#02015C0000000000 # 5C: Oil temperature
  sleep 0.05
done
