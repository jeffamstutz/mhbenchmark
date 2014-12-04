#!/bin/bash

#usage [scene directory] [width] [height] [image name suffix]

mkdir -p images

echo "--------------------"
echo " MH Benchmark suite"
echo "--------------------"

echo " "
echo "-->conf:"
echo " "
./benchmark $1/conf/conf.obj $2 $3 images/conf_$4.png

echo " "
echo "-->fair:"
echo " "
./benchmark $1/fair/fair.obj $2 $3 images/fair_$4.png

echo " "
echo "-->hairball:"
echo " "
./benchmark $1/hairball/hairball.obj $2 $3 images/hairball_$4.png

echo " "
echo "-->pplant:"
echo " "
./benchmark $1/pplant/pplant.obj $2 $3 images/pplant_$4.png

echo " "
echo "-->sanm:"
echo " "
./benchmark $1/sanm/sanm.obj $2 $3 images/sanm_$4.png

echo " "
echo "-->sibe:"
echo " "
./benchmark $1/sibe/sibe.obj $2 $3 images/sibe_$4.png

echo " "
echo "-->tank:"
echo " "
./benchmark $1/tank/tank.obj $2 $3 images/tank_$4.png

echo " "
echo "-->truck:"
echo " "
./benchmark $1/truck/truck.obj $2 $3 images/truck_$4.png
