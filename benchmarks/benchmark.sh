#!/bin/sh
clear
echo "Running benchmarks"
./fixedK 2d_simple_empty & ./rrbt 2d_simple_empty & ./varyK 2d_simple_empty

./fixedK 2d_simple_block & ./rrbt 2d_simple_block & ./varyK 2d_simple_block

./fixedK 2d_simple_narrow  & ./rrbt 2d_simple_narrow & ./varyK 2d_simple_narrow

./fixedK 2d_simple_underwater & ./rrbt 2d_simple_underwater & ./varyK 2d_simple_underwater
