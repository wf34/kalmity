#!/usr/bin/env bash

ffmpeg -r 33 -i frames/%07d.png -vcodec libx264 -preset slow -crf 18 output.mp4
