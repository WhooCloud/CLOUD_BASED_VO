#!/bin/bash
echo $1
cd build
make
cd ..
./bin/$1
