#!/bin/bash

TIME=60
FILE=./grab_FireflyMV

timeout $TIME $FILE | grep While | awk '{
	if( $1 == "While" &&
		$2 == "loop"  &&
		$3 == "time:" &&
		$4 > 0) 
		{print $4};
}'
