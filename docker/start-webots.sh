#!/bin/sh
set -eu
touch /tmp/webots-started
mkdir -p "$WEB_STATE_DIR"
# A fresh deployment must wait for a mission instead of generating a survey.
if [ ! -f "$WEB_STATE_DIR/route.csv" ]; then
  printf 'x,y,headingDeg\n' > "$WEB_STATE_DIR/route.csv"
fi
if [ ! -f "$WEB_STATE_DIR/limit_zones.txt" ]; then
  printf 'zone_count 0\n' > "$WEB_STATE_DIR/limit_zones.txt"
fi
if [ ! -f "$WEB_STATE_DIR/surface_zones.txt" ]; then
  printf 'surface_zone_count 0\n' > "$WEB_STATE_DIR/surface_zones.txt"
fi
exec xvfb-run --auto-servernum --server-args="-screen 0 1280x720x24" \
  webots --batch --stdout --stderr --mode=realtime --stream=mjpeg \
  /project/worlds/youbot_only.wbt
