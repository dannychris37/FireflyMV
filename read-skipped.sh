#!/bin/bash

TIME=3600
EXEC=./grab_FireflyMV
FILE=readings/8cams-skipped-sequential2.txt
RERUN=0

if [ $RERUN -eq 1 ]
then
	timeout $TIME $EXEC | grep "camera\|skipped" > $FILE
fi

cat $FILE | sed $'s|^\(On camera no. 0\)|\f\\1|' | 
awk 'BEGIN {cnt=0; RS="\f"; FS="\n"} /skipped:79/ {
	if($1!="On camera no. 0" || 
		$2!="On camera no. 1" || 
	$3!="On camera no. 2" || 
	$4!="On camera no. 3" || 
	$5!="skipped:80" || 
	$6!="On camera no. 4" || 
	$7!="On camera no. 5" || 
	$8!="skipped:52" || 
	$9!="On camera no. 6" || 
	$10!="skipped:78" || 
	$11!="On camera no. 7" || 
	$12!="skipped:79") {
		cnt=cnt+1; 
		print ("occurence "cnt); 
		print}
	}'