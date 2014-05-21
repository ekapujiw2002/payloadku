#!/bin/bash

#check_process() {
  #echo "$ts: checking $1"
  #[ "$1" = "" ]  && return 0
  #[ `pgrep -n $1` ] && return 1 || return 0
#}
CFG_FILE="/dev/shm/cfg.ini"

#run payload if not yet running
if [ "`pidof -s payload`" ]; then
	echo "Payload already running with PID=`pidof -s payload`"
else
	#copy cfg.ini to /dev/shm if not exists
	if [ ! -f "$CFG_FILE" ];
	then
		echo "Configuration file not found. Copy it..."
		`cp /home/raspi/komurindo/cfg.ini "$CFG_FILE"`
	fi

	#recheck
	if [ -f "$CFG_FILE" ];
	then
		echo "Configuration file loaded from $CFG_FILE"
	fi

	echo "Starting payload. Please wait..."
	`sudo /home/raspi/komurindo/payload "/dev/shm" > /dev/null 2>&1 &`
	sleep 5
	echo "Payload started with PID=`pidof -s payload`"
fi
