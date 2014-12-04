#!/bin/bash

#usage [scene directory] [width] [height] [image name suffix]

mkdir -p images

echo "--------------------"
echo " MH Benchmark suite"
echo "--------------------"

echo " "
echo "-->conf:"
echo " "
#./benchmark $1/conf.obj $2 $3 images/conf_$4.png
./benchmark $1/conf.obj $2 $3 images/conf_$4.png -vp 6.97425 8.7685 4.63958 -vi 0.78177 0.540631 -0.310732 -vu 0 0 1 -fv 73.7398

echo " "
echo "-->fair:"
echo " "
#./benchmark $1/fair.obj $2 $3 images/fair_$4.png
./benchmark $1/fair.obj $2 $3 images/fair_$4.png -vp 0.0556232 0.246116 0.576726 -vi 0.0582393 0.511594 -0.857251 -vu 0 0 1 -fv 46.8264

echo " "
echo "-->hairball:"
echo " "
./benchmark $1/hairball.obj $2 $3 images/hairball_$4.png

echo " "
echo "-->pplant:"
echo " "
#./benchmark $1/pplant.obj $2 $3 images/pplant_$4.png
./benchmark $1/pplant.obj $2 $3 images/pplant_$4.png -vp 101060 57838.7 167535 -vi -268184 6170.03 -97575.8 -vu -0.091375 0.993602 -0.0663811 -fv 55

echo " "
echo "-->sanm:"
echo " "
#./benchmark $1/sanm.obj $2 $3 images/sanm_$4.png
./benchmark $1/sanm.obj $2 $3 images/sanm_$4.png -vp -1.89777 1.5764 25.4448 -vi 0.0502237 -0.0990278 -0.993816 -u 0 1 0 -fv 73.7398

echo " "
echo "-->sibe:"
echo " "
#./benchmark $1/sibe.obj $2 $3 images/sibe_$4.png
./benchmark $1/sibe.obj $2 $3 images/sibe_$4.png -vp -19.1633 -1.50296 4.69762 -vi 0.956693 0.287606 -0.044955 -vu 0 0 1 -fv 73.7398

echo " "
echo "-->tank:"
echo " "
#./benchmark $1/tank.obj $2 $3 images/tank_$4.png
./benchmark $1/tank.obj $2 $3 images/tank_$4.png -vp 6084.42 -2333.46 992.238 -vi -3005.57 3496.19 -2070.54 -vu -0.233255 0.141724 0.962032 -fv 42.5

echo " "
echo "-->truck:"
echo " "
./benchmark $1/truck.obj $2 $3 images/truck_$4.png -vp 82.057 249.765 263.857 -vi -332.863 -85.7153 -181.053 -vu -0.376625 0.873792 -0.307638 -fv 42.5
