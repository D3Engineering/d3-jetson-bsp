#!/bin/bash
echo -e "\n ====== Xavier Multicamera Demo (16 cameras capture and display on a grid) ====== \n" 

# Configure the system
export DISPLAY=:0
sudo service nvargus-daemon stop
sudo jetson_clocks
sudo enableCamInfiniteTimeout=1 nvargus-daemon &

# Configure demo variables
CAPTURE_CAPS='video/x-raw(memory:NVMM),width=1280,height=1080'
DOWNSCALE_CAPS='video/x-raw,width=480,height=270,format=NV12,framerate=30/1'
DISPLAY_CAPS='video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12'


# Create and Launch pipeline

DEMO_1="nvarguscamerasrc sensor-id=0 name=cam1 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay1 text=camera_1 shaded-background=true ! mixer.sink_1 \
nvarguscamerasrc sensor-id=1 name=cam2 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay2 text=camera_2 shaded-background=true ! mixer.sink_2 \
nvarguscamerasrc sensor-id=2 name=cam3 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay3 text=camera_3 shaded-background=true ! mixer.sink_3 \
nvarguscamerasrc sensor-id=3 name=cam4 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay4 text=camera_4 shaded-background=true ! mixer.sink_4 \
nvarguscamerasrc sensor-id=4 name=cam5 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay5 text=camera_5 shaded-background=true ! mixer.sink_5 \
nvarguscamerasrc sensor-id=5 name=cam6 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay6 text=camera_6 shaded-background=true ! mixer.sink_6 \
nvarguscamerasrc sensor-id=6 name=cam7 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay7 text=camera_7 shaded-background=true ! mixer.sink_7 \
nvarguscamerasrc sensor-id=7 name=cam8 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay8 text=camera_8 shaded-background=true ! mixer.sink_8 \
nvarguscamerasrc sensor-id=8 name=cam9 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay9 text=camera_9 shaded-background=true ! mixer.sink_9 \
nvarguscamerasrc sensor-id=9 name=cam10 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay10 text=camera_10 shaded-background=true ! mixer.sink_10 \
nvarguscamerasrc sensor-id=10 name=cam11 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay11 text=camera_11 shaded-background=true ! mixer.sink_11 \
nvarguscamerasrc sensor-id=11 name=cam12 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay12 text=camera_12 shaded-background=true ! mixer.sink_12 \
nvarguscamerasrc sensor-id=12 name=cam13 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay13 text=camera_13 shaded-background=true ! mixer.sink_13 \
nvarguscamerasrc sensor-id=13 name=cam14 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay14 text=camera_14 shaded-background=true ! mixer.sink_14 \
nvarguscamerasrc sensor-id=14 name=cam15 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay15 text=camera_15 shaded-background=true ! mixer.sink_15 \
nvarguscamerasrc sensor-id=15 name=cam16 maxperf=true aelock=true awblock=true wbmode=0 ! $CAPTURE_CAPS ! nvvidconv ! $DOWNSCALE_CAPS ! \
queue ! textoverlay name=textoverlay16 text=camera_16 shaded-background=true ! mixer.sink_16 \
videotestsrc pattern=black ! video/x-raw,width=1,height=1 ! videomixer \
name=mixer sink_0::xpos=0 sink_0::ypos=0 sink_0::alpha=0 \
sink_1::xpos=0 sink_1::ypos=0 \
sink_2::xpos=480 sink_2::ypos=0 \
sink_3::xpos=960 sink_3::ypos=0 \
sink_4::xpos=1440 sink_4::ypos=0 \
sink_5::xpos=0 sink_5::ypos=270 \
sink_6::xpos=480 sink_6::ypos=270 \
sink_7::xpos=960 sink_7::ypos=270 \
sink_8::xpos=1440 sink_8::ypos=270 \
sink_9::xpos=0 sink_9::ypos=540 \
sink_10::xpos=480 sink_10::ypos=540 \
sink_11::xpos=960 sink_11::ypos=540 \
sink_12::xpos=1440 sink_12::ypos=540 \
sink_13::xpos=0 sink_13::ypos=810 \
sink_14::xpos=480 sink_14::ypos=810 \
sink_15::xpos=960 sink_15::ypos=810 \
sink_16::xpos=1440 sink_16::ypos=810 !
queue ! nvvidconv ! "$DISPLAY_CAPS" ! nv3dsink enable-last-sample=false sync=false async=false "

gst-launch-1.0 -v $DEMO_1


