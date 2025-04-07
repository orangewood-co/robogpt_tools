# Dalus AI Library For GS Rendering - BETA

This library allows users to connect gaussian splats (GS) with individual simulation objects and create photorealistic renders in real time.

Tested on Ubuntu 22.04 on a 8 GB NVIDIA NVIDIA GeForce RTX 4060 Ti GPU.

## How to get started
See `/isaac-sim/dalus-isaac-sim-lib/orangewood_demo.py` for an example on how to use our library to render GS in your Isaac Sim simulation.

```bash
cd /isaac-sim/dalus-isaac-sim-lib/
micromamba activate /isaac-sim/y/envs/gsplat
source /isaac-sim/setup_conda_env.sh
python run_simulation.py
```

## Notes / Troubleshooting

### Display 
In some Linux distros and Isaac Sim versions you may see the following error when starting up a simulation (e.g. when running `orangewood_demo.py`):

```bash
[Error] [omni.appwindow.plugin] Failed to acquire IWindowing interface
```

If that's the case, then try the following:
```bash
# First run this outside of the docker container
xhost local:root
# Then see what's set for $DISPLAY
echo $DISPLAY
# if it's :1 then set it to :2, if it's :2 then set it to :1, e.g.
export DISPLAY=:2
```

### Camera
If you start Isaac Sim and start moving around your simulation and don't see the Dalus window moving then in the Isaac Sim GUI select `Perspective -> Cameras -> Camera`