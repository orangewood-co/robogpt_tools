#!/bin/bash

xhost local:root
XAUTH=/tmp/.docker.xauth

docker stop dalus-owl && docker rm dalus-owl
