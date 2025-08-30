#!/bin/sh
clear
echo "Running benchmarks"
# ./fixedK 2d_simple_empty  & ./varyK 2d_simple_empty

# ./fixedK 2d_simple_block & ./varyK 2d_simple_block

# ./fixedK 2d_simple_narrow & ./varyK 2d_simple_narrow

# ./fixedK 2d_simple_underwater & ./varyK 2d_simple_underwater


# ./rrbt 2d_simple_empty
# ./rrbt 2d_simple_block
# ./rrbt 2d_simple_narrow
# ./rrbt 2d_simple_underwater

./double 2d_double_block_10 & ./double 2d_double_block_09
./double 2d_double_block_08 & ./double 2d_double_block_07
./double 2d_double_block_05 & ./double 2d_double_block_06
./double 2d_double_block_03 & ./double 2d_double_block_04
./double 2d_double_block_01 & ./double 2d_double_block_02

./double 2d_double_narrow_02 & ./double 2d_double_block
./double 2d_double_narrow & ./double 2d_double_narrow_01
./double 2d_double_narrow_03 & ./double 2d_double_narrow_04
./double 2d_double_narrow_05 & ./double 2d_double_narrow_06
./double 2d_double_narrow_07 & ./double 2d_double_narrow_08
./double 2d_double_narrow_09 & ./double 2d_double_narrow_10

./double 2d_double_underwater & ./double 2d_double_underwater_01

./double 2d_double_underwater_02 & ./double 2d_double_underwater_03
./double 2d_double_underwater_04 & ./double 2d_double_underwater_05
./double 2d_double_underwater_06 & ./double 2d_double_underwater_07
./double 2d_double_underwater_08 & ./double 2d_double_underwater_09
./double 2d_double_underwater_10

# ./varyK 2d_unicycle_underwater & ./varyK 2d_unicycle_underwater_01
# ./varyK 2d_unicycle_underwater_02 & ./varyK 2d_unicycle_underwater_03
# ./varyK 2d_unicycle_underwater_04 & ./varyK 2d_unicycle_underwater_05
# ./varyK 2d_unicycle_underwater_06 & ./varyK 2d_unicycle_underwater_07
# ./varyK 2d_unicycle_underwater_08 & ./varyK 2d_unicycle_underwater_09
# ./varyK 2d_unicycle_underwater_10

# ./varyK 2d_unicycle_narrow & ./varyK 2d_unicycle_narrow_01
# ./varyK 2d_unicycle_narrow_02 & ./varyK 2d_unicycle_narrow_03
# ./varyK 2d_unicycle_narrow_04 & ./varyK 2d_unicycle_narrow_05
# ./varyK 2d_unicycle_narrow_06 & ./varyK 2d_unicycle_narrow_07
# ./varyK 2d_unicycle_narrow_08 & ./varyK 2d_unicycle_narrow_09
# ./varyK 2d_unicycle_narrow_10

# ./varyK 2d_unicycle_block & ./varyK 2d_unicycle_block_01
# ./varyK 2d_unicycle_block_02 & ./varyK 2d_unicycle_block_03
# ./varyK 2d_unicycle_block_04 & ./varyK 2d_unicycle_block_05
# ./varyK 2d_unicycle_block_06 & ./varyK 2d_unicycle_block_07
# ./varyK 2d_unicycle_block_08 & ./varyK 2d_unicycle_block_09
# ./varyK 2d_unicycle_block_10

# ./varyK 2d_simple_underwater & ./varyK 2d_simple_underwater_01
# ./varyK 2d_simple_underwater_02 & ./varyK 2d_simple_underwater_03
# ./varyK 2d_simple_underwater_04 & ./varyK 2d_simple_underwater_05
# ./varyK 2d_simple_underwater_06 & ./varyK 2d_simple_underwater_07
# ./varyK 2d_simple_underwater_08 & ./varyK 2d_simple_underwater_09
# ./varyK 2d_simple_underwater_10

./varyK 2d_simple_narrow_00 & ./varyK 2d_simple_narrow_02
# ./varyK 2d_simple_narrow_04 & ./varyK 2d_simple_narrow_06
# ./varyK 2d_simple_narrow_08 & ./varyK 2d_simple_narrow_10
# ./varyK 2d_simple_narrow_06 & ./varyK 2d_simple_narrow_07
# ./varyK 2d_simple_narrow_08 & ./varyK 2d_simple_narrow_09
# ./varyK 2d_simple_narrow_10

# ./varyK 2d_simple_block & ./varyK 2d_simple_block_01
# ./varyK 2d_simple_block_02 & ./varyK 2d_simple_block_03
# ./varyK 2d_simple_block_04 & ./varyK 2d_simple_block_05
# ./varyK 2d_simple_block_06 & ./varyK 2d_simple_block_07
# ./varyK 2d_simple_block_08 & ./varyK 2d_simple_block_09
# ./varyK 2d_simple_block_10

./varyK 2d_unicycle_narrow_00 & ./varyK 2d_unicycle_narrow_02