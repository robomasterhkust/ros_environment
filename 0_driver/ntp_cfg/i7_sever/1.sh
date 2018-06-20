#!/bin/bash

ssh -t nvidia@tx2 "echo 1 | sudo -S date --set \"$(date -u)\"" & sleep 1;

terminator -T "time" -x ssh -t nvidia@tx2 /home/nvidia/time.sh

