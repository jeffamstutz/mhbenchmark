#!/bin/bash

#usage [scene directory] [width] [height] [image name suffix]

echo "--------------------"
echo " MH Benchmark suite"
echo "--------------------"

echo " "
echo "-->hairball:"
echo " "
./benchmark $1/hairball/hairball.obj $3 $4 images/hairball_$2.png

echo " "
echo "-->powerplant:"
echo " "
./benchmark $1/powerplant/powerplant.obj $3 $4 images/powerplant_$2.png
