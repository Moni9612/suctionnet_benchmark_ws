#!/bin/bash

# Set the CUDA device for inference
export CUDA_VISIBLE_DEVICES=0

# Run the inference script with the specified parameters
python3 /home/moniesha/suctionnet-baseline/neural_network/inference.py \
  --model deeplabv3plus_resnet101 \
  --checkpoint_path /home/moniesha/suctionnet-baseline/neural_network/example_data/realsense-deeplabplus-RGBD \
  --split test_novel \
  --camera realsense \
  --dataset_root /home/moniesha/suctionnet-baseline/neural_network/example_data/test_novel \
  --save_dir /home/moniesha/suctionnet-baseline/neural_network/example_data/save_data \
  --save_visu

