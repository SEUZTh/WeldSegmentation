cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(EX1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread")
set(CMAKE_BUILD_TYPE Debug)

find_package(PCL 1.12 REQUIRED)
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    message(PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
else()
    message(err: PCL not found)
endif()

find_package(OpenCV 4.0.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS serialization system filesystem program_options thread)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
    message(Boost_LIBRARY_DIRS = ${Boost_LIBRARY_DIRS})
    message(Boost_THREAD_LIBRARY = ${Boost_THREAD_LIBRARY})
    add_definitions(-DBOOST_ALL_DYN_LINK)
else()
    message(err: Boost not found)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)

set(SRC 
    src/normal_correction.cpp
    src/plane_segment.cpp
    src/pre_process.cpp
    src/visualization.cpp)

add_executable(main main.cpp ${SRC})
target_link_libraries(main
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    )

add_executable(test test.cpp)
target_link_libraries(test
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    )