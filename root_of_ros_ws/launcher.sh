#!/bin/bash

EXIT_NUM=(0 0 0 0 0 0)
TOTAL_LENGTH=0.0

echo " " > launcher_log

trap "exit" INT
for i in {1..300}
do
	echo iteration ":" $i 
	roslaunch pointcloudTraj clean_demo.launch &>> launcher_log

	cat exit_code
	cat traj_length

	source exit_code
	source traj_length

	EXIT_NUM[$EXIT_CODE]=$((${EXIT_NUM[$EXIT_CODE]}+1))

	if [ "$EXIT_CODE" -eq 0 ]
	then
		TOTAL_LENGTH=$(echo "$TOTAL_LENGTH + $TRAJ_LENGTH" | bc -l)
	fi
done

for i in {0..5} 
do
	echo ${EXIT_NUM[i]}	
done

if [ "${EXIT_NUM[0]}" -ne 0 ]
then
	AVERAGE_LENGTH=$(echo "$TOTAL_LENGTH / ${EXIT_NUM[0]}" | bc -l)	
	echo average = $AVERAGE_LENGTH
else
	echo average = 0
fi

#roslaunch pointcloudTraj clean_demo.launch 2>&1 | tee out
