cmake_minimum_required(VERSION 3.16)
# A fixed version keeps the app description, and so the first image sector, unchanged between builds.
set(PROJECT_VER "1")
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drone)
