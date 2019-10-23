#!/bin/bash

TIME=60
FILE=./grab_FireflyMV

timeout $TIME $FILE | grep waiting | awk '{
	if( $4 > 0 ) print $4; 
}'