#!/bin/bash

last_pwd=$( pwd )
project_dir=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd $project_dir

[ ! -d "build" ] && mkdir build
cd build
cmake ..
cmake --build .

