#!/bin/bash

N=100
OUTFILE="ct_solution_barrier_rrt_cost.txt"

# Clear old summary file
> $OUTFILE

for i in $(seq 1 $N); do
    echo "Run $i..."
    ./build/barrier_rrt config/2d_barrier_empty.ini > ct_tmp_output.txt

    # Extract cost
    COST=$(grep "Path cost" ct_tmp_output.txt | awk '{print $4}')

    # Extract planning time (strip "ms")
    TIME=$(grep "Planning time" ct_tmp_output.txt | awk '{print $3}')

    # Save "COST TIME" per run
    echo "$COST $TIME" >> $OUTFILE
done

# Compute averages
awk '{cost+=$1; time+=$2} END {
    print "Average cost over " NR " runs:", cost/NR
    print "Average planning time (ms) over " NR " runs:", time/NR
}' $OUTFILE

