#!/bin/bash

#usage [scene directory] [width] [height] [image name suffix]

mkdir -p images

echo "--------------------"
echo " MH Benchmark suite"
echo "--------------------"

echo " "
echo "-->hairball:"
echo " "
./benchmark $1/hairball/hairball.obj $2 $3 images/hairball_$4.png

echo " "
echo "-->powerplant:"
echo " "
./benchmark $1/powerplant/powerplant.obj $2 $3 images/powerplant_$4.png
