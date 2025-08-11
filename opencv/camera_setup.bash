#!/bin/bash

# Disable all automatic features
v4l2-ctl -c focus_automatic_continuous=0 -d 4
v4l2-ctl -c white_balance_automatic=0 -d 4
v4l2-ctl -c auto_exposure=1 -d 4 # This is manual mode - not enabled

# Set camera parameters
v4l2-ctl -c focus_absolute=0 -d 4
v4l2-ctl -c brightness=128 -d 4
v4l2-ctl -c sharpness=128 -d 4
v4l2-ctl -c contrast=128 -d 4
v4l2-ctl -c saturation=128 -d 4