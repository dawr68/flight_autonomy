cmake_minimum_required(VERSION 3.0.2)
project(flight_autonomy)

## Compile as C++11,
add_compile_options(-std=c++11)

set(OpenCV_DIR /usr/share/OpenCV/)
set(MAVSDK_DIR /usr/lib/cmake/MAVSDK/)

find_package(
  MAVSDK REQUIRED
  OpenCV REQUIRED
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/Flight-autonomy.cpp
# )

## Declare a C++ executable
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/main.cpp src/FlightAutonomy.cpp src/ImageReceiver.cpp src/FlightControl.cpp src/ObjectDetector.cpp)



## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  MAVSDK::mavsdk_telemetry
  MAVSDK::mavsdk_offboard
  MAVSDK::mavsdk_info
  MAVSDK::mavsdk_action
  MAVSDK::mavsdk
)
