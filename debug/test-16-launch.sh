#!/bin/bash

echo -e "\n ===== Xavier Multicamera Demo (16 cameras capture and display on a grid) ===== \n"

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
nvarguscamerasrc sensor-id=4 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_4 \
nvarguscamerasrc sensor-id=5 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_5 \
nvarguscamerasrc sensor-id=6 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_6 \
nvarguscamerasrc sensor-id=7 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_7 \
nvarguscamerasrc sensor-id=8 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_8 \
nvarguscamerasrc sensor-id=9 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_9 \
nvarguscamerasrc sensor-id=10 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_10 \
nvarguscamerasrc sensor-id=11 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_11 \
nvarguscamerasrc sensor-id=12 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_12 \
nvarguscamerasrc sensor-id=13 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_13 \
nvarguscamerasrc sensor-id=14 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_14 \
nvarguscamerasrc sensor-id=15 maxperf=true ! "$CAPTURE_CAPS" ! mqueue.sink_15 \
mqueue.src_0 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*0)) window-y=$((0)) \
mqueue.src_1 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*1)) window-y=$((0)) \
mqueue.src_2 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*2)) window-y=$((0)) \
mqueue.src_3 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*3)) window-y=$((0)) \
mqueue.src_4 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*0)) window-y=$(($HEIGHT*1)) \
mqueue.src_5 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*1)) window-y=$(($HEIGHT*1)) \
mqueue.src_6 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*2)) window-y=$(($HEIGHT*1)) \
mqueue.src_7 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*3)) window-y=$(($HEIGHT*1)) \
mqueue.src_8 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*0)) window-y=$(($HEIGHT*2)) \
mqueue.src_9 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*1)) window-y=$(($HEIGHT*2)) \
mqueue.src_10 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*2)) window-y=$(($HEIGHT*2)) \
mqueue.src_11 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*3)) window-y=$(($HEIGHT*2)) \
mqueue.src_12 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*0)) window-y=$(($HEIGHT*3)) \
mqueue.src_13 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*1)) window-y=$(($HEIGHT*3)) \
mqueue.src_14 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*2)) window-y=$(($HEIGHT*3)) \
mqueue.src_15 ! nvvidconv ! "$DOWNSCALE_CAPS" ! nvvidconv ! nv3dsink window-x=$(($WIDTH*3)) window-y=$(($HEIGHT*3))
