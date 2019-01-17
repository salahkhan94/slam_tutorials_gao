cmake_minimum_required(VERSION 2.8)
project( gaoslam )

add_library( gaoslam SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
)

target_link_libraries( gaoslam
    ${THIRD_PARTY_LIBS}
)
