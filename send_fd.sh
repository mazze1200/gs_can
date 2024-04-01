#!/bin/bash
# link https://munich.dissec.to/kb/chapters/can/canfd.html

while true
do
	# cansend can0 222##6010203040506070809
	cansend can0 444##1deadbeefdeadbeefdeadbeefdeadbeef  # Send a 16 bytes CAN FD message with an higher bitrate for the data
done
