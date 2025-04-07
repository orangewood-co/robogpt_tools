# GSPLAT Environment Setup

## Install micromamba if not installed
```bash
"${SHELL}" <(curl -L micro.mamba.pm/install.sh)
```

## Create env
```bash
micromamba create -n gsplat python=3.10 -c conda-forge -c conda
micromamba activate gsplat
```

## Install CUDA Toolkit inside env
```bash
micromamba install cuda -c nvidia/label/cuda-12.2.0 # replace with your desired CUDA Toolkit version
```
## Update environment vars
```bash
export PATH=<path_to_micromamba_root>/envs/gsplat/bin:$PATH
export LD_LIBRARY_PATH=<path_to_micromamba_root>/envs/gsplat/lib:$LD_LIBRARY_PATH
```

## Install dependencies (in this order)
```bash
micromamba install pytorch torchvision pytorch-cuda=12.1 -c pytorch -c nvidia
pip install git+https://github.com/nerfstudio-project/gsplat.git # this will take some time
cd DalusSimCoreRelease/
pip install -r requirements.txt
```
