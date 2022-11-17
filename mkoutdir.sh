#!/bin/bash

root=/home/dlr/Project1/ORB_SLAM2_Enhanced

evalset=(
    # office0
    office1
    office2
    office3
    office4
    room0
    room1
    room2
)

for ((i=0; i<8; i++)); do
    # printf "now do Replica-%s \t \n" "${evalset[$i]}"
    # CUDA_VISIBLE_DEVICES=3 python -W ignore run.py configs/Replica/${evalset[$i]}.yaml >log/Replica-${evalset[$i]}.log 
    # nohup ./eval_replica.sh >log/eval_replica.log 2>&1 &
    mkdir -p result/replica/${evalset[$i]}/keypt

    # 运行不含窗口的
    $root/Examples/Monocular/mono_replica $root/Vocabulary/ORBvoc.bin $root/Examples/Monocular/Replica-${evalset[$i]}.yaml $root/dataset/Replica/${evalset[$i]} 0
done