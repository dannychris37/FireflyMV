#!/bin/bash

TIME=60
FILE=./grab_FireflyMV

timeout $TIME $FILE | \
awk '/waiting/{
	if( $1 == "Cam" && 
		($2 == "0"  || 
		$2 == "1" || 
		$2 == "2" || 
		$2 == "3" || 
		$2 == "4" || 
		$2 == "5" ||
		$2 == "6" ||
		$2 == "7") && 
		$3 == "frame" && 
		$4 == "waiting" && 
		$5 == "time:" && 
		$6 != "Cam" && 
		$6 > 0) 
		{print};
}'