#!/bin/bash

clean () {
  rm -rf build/ install/ log/
}

usage () {
  echo "Usage: $0 [-c]"
  echo -e "Builds Milrem AS test work\n"
  echo "Optional arguments:"
  echo -e "  -c\tClean build"
  exit 0
}

build () {
  mv CMakeLists.txt CMakeLists.txt.old
  colcon build --merge-install
  mv CMakeLists.txt.old CMakeLists.txt
}

while getopts "c" flag
do
    case "${flag}" in
        c) clean;;
        *) usage;;
    esac
done

build