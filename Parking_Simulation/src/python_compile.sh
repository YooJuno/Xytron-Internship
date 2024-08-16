#!/bin/bash

names=("simulator")

#sudo apt install cython

for k in "${names[@]}"; 
do 
   echo $k
   cython -a $k.py
   gcc -shared -pthread -fPIC -fwrapv -O2 -Wall -fno-strict-aliasing -I/usr/include/python3.8 -lpython3.8  -o $k.so $k.c
done

sudo rm -rf ./*.c*
sudo rm -rf ./*.html
