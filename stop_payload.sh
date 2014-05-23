#!/bin/bash
if [ "`pidof -s payload`" ]; then
	echo "Stopping payload with PID = `pidof -s payload`............"
	sudo pkill payload
else
	echo "Payload already stopped"
fi
