PS1="[PBPF] Singularity> \w \$ "
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=$(hostname)
export XLA_FLAGS=--xla_gpu_cuda_data_dir=/home/zisongxu/.local/lib/python3.8/site-packages/nvidia/cuda_nvcc

export PYTHONPATH=$HOME/pyvkdepth/bin:$PYTHONPATH
