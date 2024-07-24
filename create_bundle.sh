#!/bin/bash

rm -f homework.tar.gz

tar -czvf homework.tar.gz \
    src \
    app_launch.py \
    build.sh \
    CMakeLists.txt \
    DOCUMENTATION.md \
    README.md \
    sensor_sender.py