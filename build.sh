#!/bin/bash

clean () {
  rm -rf build/ install/ log/
}

usage () {
  echo "Usage: $0 [-c] [-a <args>]"
  echo -e "Builds Milrem AS test work\n"
  echo "Optional arguments:"
  echo -e "  -c       \tClean build"
  echo -e "  -a <args>\tSet CMAKE args (e.g.-DCMAKE_BUILD_TYPE=Debug)"
  exit 0
}

build () {
  mv CMakeLists.txt CMakeLists.txt.old
  colcon build --merge-install ${CMAKE_ARGS} || { echo "Build failed"; mv CMakeLists.txt.old CMakeLists.txt; exit 1; }
  mv CMakeLists.txt.old CMakeLists.txt
}

CMAKE_ARGS=""

while getopts "ca:" flag
do
    case "${flag}" in
        c) clean;;
        a) CMAKE_ARGS="--cmake-args ${CMAKE_ARGS} $OPTARG";;
        *) usage;;
    esac
done

build