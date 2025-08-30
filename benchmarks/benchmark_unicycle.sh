#!/bin/sh
clear
echo "Running benchmarks"
# ./fixedK 2d_unicycle_empty & ./varyK 2d_unicycle_empty

# ./fixedK 2d_unicycle_block & ./rrbt 2d_unicycle_block & ./varyK 2d_unicycle_block

./rrbt 2d_unicycle_block
./rrbt 2d_unicycle_narrow
./rrbt 2d_unicycle_underwater

# ./fixedK 2d_unicycle_narrow  & ./varyK 2d_unicycle_narrow
# ./fixedK 2d_unicycle_underwater & ./varyK 2d_unicycle_underwater
# 
# ./fixedK 2d_unicycle_underwater & ./rrbt 2d_unicycle_underwater & ./varyK 2d_unicycle_underwater

