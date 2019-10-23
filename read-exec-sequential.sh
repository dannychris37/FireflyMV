#!/bin/bash

TIME=60
FILE=./grab_FireflyMV

timeout $TIME $FILE | grep while | awk '{
	if( $4 > 0 ) print $4; 
}'