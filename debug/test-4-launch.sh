#!/bin/bash

echo -e "\n ===== Xavier Multicamera Demo (4 cameras capture and display on a grid) ===== \n"

# Configure system
export DISPLAY=:0
sudo service nvargus-daemon stop
sudo jetson_clocks
sudo enableCamInfiniteTimeout=1 nvargus-daemon &

# Configure pipeline variables
CAPTURE_CAPS='video/x-raw(memory:NVMM),width=(int)1280,height=(int)1080,format=(string)NV12,framerate=(fraction)30/1'
DOWNSCALE_CAPS='video/x-raw,memory=NVMM,width=360,height=260,format=NV12'
WIDTH=360
HEIGHT=260

gst-launch-1.0 -v \
multiqueue max-size-buffers=100 name=mqueue \
nvarguscamerasrc sensor-id=0 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_0 \
nvarguscamerasrc sensor-id=1 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_1 \
nvarguscamerasrc sensor-id=2 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_2 \
nvarguscamerasrc sensor-id=3 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_3 \
mqueue.src_0 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*0)) window-y=$((0)) \
mqueue.src_1 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*1)) window-y=$((0)) \
mqueue.src_2 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*2)) window-y=$((0)) \
mqueue.src_3 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*3)) window-y=$((0)) \
