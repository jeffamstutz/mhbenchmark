#!/bin/bash

# usage:  run_tests.sh <scene_dir> <width> <height> <image name suffix>

if [ -z $1 ]; then echo "usage:  run_tests.sh <scene_dir> <width> <height> <image name suffix>"; exit; fi
if [ -z $2 ]; then echo "usage:  run_tests.sh <scene_dir> <width> <height> <image name suffix>"; exit; fi
if [ -z $3 ]; then echo "usage:  run_tests.sh <scene_dir> <width> <height> <image name suffix>"; exit; fi
if [ -z $4 ]; then echo "usage:  run_tests.sh <scene_dir> <width> <height> <image name suffix>"; exit; fi

mkdir -p images

echo "--------------------"
echo " MH Benchmark suite"
echo "--------------------"

echo " "
echo "-->conf:"
echo " "
#bin/benchmark $1/conf.obj $2 $3 images/conf_$4.png -vp 1.02501 2.08907 8.07266 -vi 12.8822 11.094 1.67948 -vu 0 0 1 -fv 73.7398 --osp:coi
bin/benchmark $1/conf.obj $2 $3 images/conf_$4.png `cat $1/conf.ebv` --osp:coi

echo " "
echo "-->fair:"
echo " "
#bin/benchmark $1/fair.obj $2 $3 images/fair_$4.png -vp -0.659629 1.13712 0.863232 -vi 0.354431 -0.204038 -0.722141 -vu 0 1 0 -fv 46.8264 --osp:coi
bin/benchmark $1/fair.obj $2 $3 images/fair_$4.png `cat $1/fair.ebv` --osp:coi

echo " "
echo "-->hairball:"
echo " "
#bin/benchmark $1/hairball.obj $2 $3 images/hairball_$4.png
bin/benchmark $1/hairball.obj $2 $3 images/hairball_$4.png `cat $1/hairball.ebv` --osp:coi

echo " "
echo "-->pplant:"
echo " "
#bin/benchmark $1/pplant.obj $2 $3 images/pplant_$4.png -vp 101060 57838.7 167535 -vi -268184 6170.03 -97575.8 -vu -0.091375 0.993602 -0.0663811 -fv 80.0 --osp:coi
bin/benchmark $1/pplant.obj $2 $3 images/pplant_$4.png `cat $1/pplant.ebv` --osp:coi

echo " "
echo "-->sanm:"
echo " "
#bin/benchmark $1/sanm.obj $2 $3 images/sanm_$4.png -vp -1.89777 1.5764 25.4448 -vi 0.0502237 -0.0990278 -0.993816 -u 0 1 0 -fv 73.7398 --osp:coi
bin/benchmark $1/sanm.obj $2 $3 images/sanm_$4.png `cat $1/sanm.ebv` --osp:coi

echo " "
echo "-->sibe:"
echo " "
#bin/benchmark $1/sibe.obj $2 $3 images/sibe_$4.png -vp -19.1633 -1.50296 4.69762 -vi 0.956693 0.287606 -0.044955 -vu 0 0 1 -fv 73.7398 --osp:coi
bin/benchmark $1/sibe.obj $2 $3 images/sibe_$4.png `cat $1/sibe.ebv` --osp:coi

echo " "
echo "-->tank:"
echo " "
#bin/benchmark $1/tank.obj $2 $3 images/tank_$4.png -vp 6084.42 -2333.46 992.238 -vi -3005.57 3496.19 -2070.54 -vu -0.233255 0.141724 0.962032 -fv 60.0 --osp:coi
bin/benchmark $1/tank.obj $2 $3 images/tank_$4.png `cat $1/tank.ebv` --osp:coi

echo " "
echo "-->truck:"
echo " "
#bin/benchmark $1/truck.obj $2 $3 images/truck_$4.png -vp 82.057 249.765 263.857 -vi -332.863 -85.7153 -181.053 -vu -0.376625 0.873792 -0.307638 -fv 60.0 --osp:coi
bin/benchmark $1/truck.obj $2 $3 images/truck_$4.png `cat $1/truck.ebv` --osp:coi
