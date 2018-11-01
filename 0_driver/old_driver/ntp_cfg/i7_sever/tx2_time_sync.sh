#!/bin/bash

ssh -t dji@10.0.0.1 "echo dji | sudo -S date --set \"$(date -u)\"" & sleep 1;

terminator -T "time" -x ssh -t dji@10.0.0.1 /home/dji/time.sh

