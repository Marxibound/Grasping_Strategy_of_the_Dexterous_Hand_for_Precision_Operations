#!/bin/bash

EXP_NAME=$1
MAIN_GPU=${2:-0}

# Auto detect available GPUs and port
export MASTER_PORT=$(comm -23 <(seq 29500 29510 | sort) <(ss -Htan | awk '{print $4}' | cut -d':' -f2 | sort -u) | shuf -n 1)
export CUDA_VISIBLE_DEVICES=$(nvidia-smi --query-gpu=index --format=csv,noheader | awk -v start=$MAIN_GPU 'NR>=start+1' | paste -sd "," -)

# Dynamic GPU params
NUM_GPU=$(echo $CUDA_VISIBLE_DEVICES | tr -cd , | wc -c)
[ $NUM_GPU -eq 0 ] && { echo "No available GPUs"; exit 1; }
let NUM_GPU++

# Core training command
torchrun --nproc_per_node=$NUM_GPU \
    --rdzv_id=${SLURM_JOB_ID} --rdzv_backend=c10d \
    --rdzv_endpoint=localhost:$MASTER_PORT \
    train_ddm.py \
        hydra/job_logging=none \
        hydra/hydra_logging=none \
        exp_name=${EXP_NAME} \
        diffuser=ddpm \
        diffuser.loss_type=l1 \
        diffuser.steps=100 \
        model=unet_grasp \
        task=grasp_gen_ur \
        task.dataset.normalize_x=true \
        task.dataset.normalize_x_trans=true