cmake_minimum_required(VERSION 2.8)
project(salSLAM)

add_executable( salslam run_vo.cpp )
target_link_libraries( salslam )
