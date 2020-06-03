#!/bin/bash

EXIT_NUM=(0 0 0 0 0 0)
TOTAL_LENGTH=0.0

ITER_NUM=$2

printf "iter code length\n"

trap "exit" INT
for ((i=1;i<=ITER_NUM;i++))
do
	printf "%4d " $i
	roslaunch pointcloudTraj clean_demo.launch $1 &>> launcher_log

	source exit_code
	source traj_length

	printf "%4d " $EXIT_CODE
	echo $TRAJ_LENGTH

	EXIT_NUM[$EXIT_CODE]=$((${EXIT_NUM[$EXIT_CODE]}+1))

	if [ "$EXIT_CODE" -eq 0 ]
	then
		TOTAL_LENGTH=$(echo "$TOTAL_LENGTH + $TRAJ_LENGTH" | bc -l)
	fi
done

echo ${EXIT_NUM[0]} ${EXIT_NUM[1]} ${EXIT_NUM[2]} ${EXIT_NUM[3]} ${EXIT_NUM[4]} ${EXIT_NUM[5]}

if [ "${EXIT_NUM[0]}" -ne 0 ]
then
	AVERAGE_LENGTH=$(echo "$TOTAL_LENGTH / ${EXIT_NUM[0]}" | bc -l)	
	echo average : $AVERAGE_LENGTH
else
	echo average : 0
fi
