#!/usr/bin/env bash

docker build \
    --build-arg EXPERIMENTAL_ZENOH_RMW=TRUE \
    -t chiyunnn/trail-odom:latest \
    ./
