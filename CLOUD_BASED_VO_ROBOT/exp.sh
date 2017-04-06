#!/bin/bash
for i in $(seq 1 $1)
do
./bin/robot > ./log/robot$1_$i.log &
done
