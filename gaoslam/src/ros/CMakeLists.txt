cmake_minimum_required(VERSION 2.8)
project(gaoslam)

add_executable( runslam run_vo.cpp )
target_link_libraries( runslam ${THIRD_PARTY_LIBS}  ${catkin_LIBRARIES} gaoslam)
