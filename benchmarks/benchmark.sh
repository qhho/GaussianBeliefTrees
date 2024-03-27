#!/bin/sh
clear
echo "Running benchmarks"
./benchmark_2dsimple 2d_simple_empty & ./benchmark_2drrbt 2d_simple_empty & ./benchmark_2dsimple_fixedK 2d_simple_empty

./benchmark_2dsimple 2d_simple_block & ./benchmark_2drrbt 2d_simple_block & ./benchmark_2dsimple_fixedK 2d_simple_block

./benchmark_2dsimple 2d_simple_narrow  & ./benchmark_2drrbt 2d_simple_narrow & ./benchmark_2dsimple_fixedK 2d_simple_narrow

./benchmark_2dsimple 2d_simple_underwater & ./benchmark_2drrbt 2d_simple_underwater & ./benchmark_2dsimple_fixedK 2d_simple_underwater
