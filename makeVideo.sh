#!/bin/bash

if [ $# -le 3 ]; then 
    echo -e "To make the video specify the following arguments:"
    echo -e "$0 <input-path> <extension> <output-file> <input-framerate>"
    echo -e "\t <input-path>: speficy the path to the images, including the file prefix except the last 5 numeric digits."
    echo -e "\t <extension>: specify the input frame file extension (without dot)"
    echo -e "\t <output-file>: specify the output filename excluding extension"
    echo -e "\t <input-framerate>: specify how many frames per seconds (5 for debug, 10 for final videos)."
    exit 1   
fi 

ffmpeg -framerate ${4} -i ${1}%05d.${2} -c:v libx264 -r 30 ${3}.mp4

