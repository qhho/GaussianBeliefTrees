#!/bin/bash

N=100
OUTFILE="dt_solution_barrier_rrt_cost.txt"
LOGFILE="dt_planner_failures.log"

# Clear old files
> $OUTFILE
> $LOGFILE

FAIL=0
SUCCESS=0

for i in $(seq 1 $N); do
    echo "Run $i..."
    ./build/discrete_continuous_example config/2d_barrier_empty.ini > dt_tmp_output.txt

    # Check if the run succeeded (look for "Found solution!")
    if grep -q "Found solution!" dt_tmp_output.txt && grep -q "Trajectory is VALID" dt_tmp_output.txt; then
        # Extract cost
        COST=$(grep "Path cost" dt_tmp_output.txt | awk '{print $4}')
        # Extract planning time (strip "ms")
        TIME=$(grep "Planning time" dt_tmp_output.txt | awk '{print $3}')
        # Save success run data
        echo "$COST $TIME" >> $OUTFILE
        SUCCESS=$((SUCCESS+1))
    else
        echo "Run $i FAILED" | tee -a $LOGFILE
        FAIL=$((FAIL+1))
    fi
done

# Compute averages only over successful runs
if [ $SUCCESS -gt 0 ]; then
    awk '{cost+=$1; time+=$2} END {
        print "Average cost over " NR " successful runs:", cost/NR
        print "Average planning time (ms) over " NR " successful runs:", time/NR
    }' $OUTFILE
fi

# Print success probability
TOTAL=$((SUCCESS+FAIL))
PSUCCESS=$(echo "scale=4; $SUCCESS/$TOTAL" | bc -l)
echo "Success probability P(success) = $PSUCCESS ($SUCCESS / $TOTAL)"
echo "Failures logged to: $LOGFILE"