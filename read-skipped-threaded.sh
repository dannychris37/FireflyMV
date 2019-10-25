#!/bin/bash

TIME=3600
EXEC=./grab_FireflyMV
FILE=readings/8cams-skipped-threaded.txt
RERUN=1

if [ $RERUN -eq 1 ]
then
	timeout $TIME $EXEC | grep "PROCESSING\|skipped" > $FILE
fi

cat $FILE | sed $'s|^\(--- PROCESSING FRAMES ---\)|\f\\1|' | 
awk 'BEGIN {cnt=0; RS="\f"; FS="\n"} /--- PROCESSING DONE ---/ {
	if(!($2 ~ /skipped:../ && $3 ~ /skipped:../ && $4 ~ /skipped:../ && $5 ~ /skipped:../)) {
		cnt=cnt+1; 
		print ("occurence "cnt); 
		print}
	}'