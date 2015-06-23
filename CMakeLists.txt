cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project (ompltest)

find_package(OMPL REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable (ompltest SE3RigidBodyPlanning.cpp)
target_link_libraries (ompltest ${OMPLAPP_LIBRARIES} ompl_app_base ${OMPL_LIBRARIES})
