#!/bin/bash
# Pepper front camera streaming script.
#
# Streams the top (head) camera as RTP/JPEG to a host on the network.
#
# WHERE IT SENDS, in precedence order:
#   1. first argument          ./start_camera.sh 10.42.0.1
#   2. PC_HOST in the env      PC_HOST=10.42.0.1 ./start_camera.sh
#   3. whoever you SSH'd in from (from SSH_CONNECTION) -- correct automatically
#      whenever you start this over SSH from the receiving laptop, on either
#      network, which is the usual case
#   4. DEFAULT_PC_HOST below
#
# ORIENTATION. The camera is mounted upside down in the head. NAOqi corrects
# that in the sensor itself, with ALVideoDevice parameters 11 (HFlip) and 12
# (VFlip), both 1 by default. Once NAOqi has configured the camera with them,
# the raw image is upright and needs no software flip. After a camera reset it
# is rotated 180 degrees instead -- and NAOqi re-applies its flips whenever it
# touches the camera again (its wake-up diagnosis does), so a stream that
# corrected in software turned upside down mid-run.
#
# The old version picked a videoflip from ALMotion.robotIsWakeUp. That was a
# proxy for which of the two raw states the sensor happened to be in, and it
# was wrong in both directions (measured 2026-11-16: upside down at rest,
# mirrored after waking, then upside down again mid-stream). Instead this
# script has NAOqi configure the sensor with its flips on right before
# releasing it, and streams with no software flip. Any later NAOqi reset
# re-applies the same flips, so the orientation no longer changes mid-run.

DEFAULT_PC_HOST="172.29.111.243"   # laptop on the CMU-Pepper router
PORT_FRONT="${PORT_FRONT:-3000}"
CAMERA=0                           # top camera
HFLIP_ID=11
VFLIP_ID=12

if [ -n "$1" ]; then
    PC_HOST="$1"
    HOST_SRC="argument"
elif [ -n "$PC_HOST" ]; then
    HOST_SRC="PC_HOST env"
elif [ -n "$SSH_CONNECTION" ]; then
    PC_HOST="${SSH_CONNECTION%% *}"
    HOST_SRC="your SSH client address"
else
    PC_HOST="$DEFAULT_PC_HOST"
    HOST_SRC="built-in default"
fi

if [ -z "$PC_HOST" ]; then
    echo "start_camera.sh: could not determine a destination host." >&2
    echo "Pass one:  ./start_camera.sh 10.42.0.1" >&2
    exit 1
fi

echo "Destination: $PC_HOST:$PORT_FRONT  (from $HOST_SRC)"

# A stream already running holds /dev/video-top; NAOqi cannot configure the
# sensor until it is gone.
pkill -f "gst-launch-0.10.*video-top" && sleep 1

# Have NAOqi open the camera, which (re)configures the sensor with its flips,
# and make sure both flips are 1. Never write 0, not even to force a rewrite:
# measured 2026-11-16, a flip write of 0 loses the lens focus and every frame
# after it is blurred (Laplacian variance 8 against 291), even once set back.
qicli call ALVideoDevice.openCamera $CAMERA > /dev/null
sleep 1
for id in $HFLIP_ID $VFLIP_ID; do
    qicli call ALVideoDevice.setParameter $CAMERA $id 1 > /dev/null
done
FLIPS="$(qicli call ALVideoDevice.getParameter $CAMERA $HFLIP_ID) $(qicli call ALVideoDevice.getParameter $CAMERA $VFLIP_ID)"
echo "NAOqi sensor flips (HFlip VFlip): $FLIPS"

# Release the camera from NAOqi
qicli call ALVideoDevice.closeCamera $CAMERA > /dev/null
sleep 2

# Start front camera stream (background). No videoflip: the sensor is upright.
gst-launch-0.10 -v v4l2src device=/dev/video-top ! \
'video/x-raw-yuv,width=640,height=480,framerate=30/1' ! \
ffmpegcolorspace ! \
jpegenc quality=85 ! rtpjpegpay ! \
udpsink host=$PC_HOST port=$PORT_FRONT > /tmp/cam_front.log 2>&1 &

echo "Front camera PID: $!"
echo "Streaming to $PC_HOST on port $PORT_FRONT"
