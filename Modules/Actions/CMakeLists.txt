CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(Actions)
FIND_PACKAGE(YARP REQUIRED)

FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
include_directories( "./include" )

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})
set(CMAKE_BUILD_TYPE "Release")


ADD_EXECUTABLE(Actions ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(Actions ${YARP_LIBRARIES} ${JSONCPP_LIBRARY} ${LINK_LIBRARIES})
