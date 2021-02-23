#!/bin/bash

sudo chmod 777 /dev/ttyUSB*
sudo systemctl restart ptp4l
sudo systemctl daemon-reload
sudo systemctl restart phc2sys
sudo systemctl status phc2sys
