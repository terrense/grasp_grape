cmake_minimum_required(VERSION 3.0.2)
project(grape_harvest)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY urdf config launch scripts worlds models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
