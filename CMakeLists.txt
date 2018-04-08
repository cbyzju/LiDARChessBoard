cmake_minimum_required(VERSION 2.8)
set(PROJECT_NAME LiDARChessBoard)
project(${PROJECT_NAME})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(${EIGEN_INCLUDE_DIRS})

file(GLOB SOURCE ./src/*.cpp ./src/.h)

add_executable(${PROJECT_NAME} ${SOURCE})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${BOOST_LIBRARIES} ${OpenCV_LIBRARIES})
