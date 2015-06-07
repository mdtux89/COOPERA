CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(SarsaPolicy)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(JsonCpp REQUIRED)
message(STATUS "json include: " ${JSONCPP_INCLUDE_DIR})
FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

##############here starts shark compilation dependent setup##########################

#map default build type to release
IF(NOT CMAKE_BUILD_TYPE)
       SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)
IF((CMAKE_BUILD_TYPE MATCHES "debug") OR (CMAKE_BUILD_TYPE MATCHES "DEBUG"))
        SET(CMAKE_BUILD_TYPE "Debug")
ENDIF((CMAKE_BUILD_TYPE MATCHES "debug") OR (CMAKE_BUILD_TYPE MATCHES "DEBUG"))
IF((CMAKE_BUILD_TYPE MATCHES "release") OR (CMAKE_BUILD_TYPE MATCHES "RELEASE"))
        SET(CMAKE_BUILD_TYPE "Release")
ENDIF((CMAKE_BUILD_TYPE MATCHES "release") OR (CMAKE_BUILD_TYPE MATCHES "RELEASE"))
#enable or disable debugging
IF(CMAKE_BUILD_TYPE MATCHES "Debug")
        ADD_DEFINITIONS(-DDEBUG )
        SET( LINK_LIBRARIES
                optimized;/usr/lib/libboost_system.a;debug;/usr/lib/libboost_system.a;optimized;/usr/lib/libboost_date_time.a;debug;/usr/lib/libboost_date_time.a;optimized;/usr/lib/libboost_filesystem.a;debug;/usr/lib/libboost_filesystem.a;optimized;/usr/lib/libboost_program_options.a;debug;/usr/lib/libboost_program_options.a;optimized;/usr/lib/libboost_signals.a;debug;/usr/lib/libboost_signals.a;optimized;/usr/lib/libboost_serialization.a;debug;/usr/lib/libboost_serialization.a;optimized;/usr/lib/libboost_thread.a;debug;/usr/lib/libboost_thread.a;pthread;optimized;/usr/lib/libboost_unit_test_framework.a;debug;/usr/lib/libboost_unit_test_framework.a
                /usr/local/lib/libshark_debug.a
        )
ENDIF (CMAKE_BUILD_TYPE MATCHES "Debug")
IF(CMAKE_BUILD_TYPE MATCHES "Release")
        ADD_DEFINITIONS(-DNDEBUG )
        ADD_DEFINITIONS(-DBOOST_UBLAS_NDEBUG )
        SET( LINK_LIBRARIES
                optimized;/usr/lib/libboost_system.a;debug;/usr/lib/libboost_system.a;optimized;/usr/lib/libboost_date_time.a;debug;/usr/lib/libboost_date_time.a;optimized;/usr/lib/libboost_filesystem.a;debug;/usr/lib/libboost_filesystem.a;optimized;/usr/lib/libboost_program_options.a;debug;/usr/lib/libboost_program_options.a;optimized;/usr/lib/libboost_signals.a;debug;/usr/lib/libboost_signals.a;optimized;/usr/lib/libboost_serialization.a;debug;/usr/lib/libboost_serialization.a;optimized;/usr/lib/libboost_thread.a;debug;/usr/lib/libboost_thread.a;pthread;optimized;/usr/lib/libboost_unit_test_framework.a;debug;/usr/lib/libboost_unit_test_framework.a
                /usr/local/lib/libshark.a
        )
ENDIF (CMAKE_BUILD_TYPE MATCHES "Release")

#handle ATLAS
#OPTION(OPT_ENABLE_ATLAS "Enable Atlas as Linear Algebra library" ON)
#mark_as_advanced(
#	OPT_ENABLE_ATLAS
#)

IF ( OPT_ENABLE_ATLAS )
        SET(LINK_LIBRARIES
                ${LINK_LIBRARIES}
                CLAPACK_LIBRARY-NOTFOUND
                CBLAS_LIBRARY-NOTFOUND
                ATLAS_LIBRARY-NOTFOUND
        )
        ADD_DEFINITIONS( -DSHARK_USE_ATLAS )
        include_directories ( /../include/ /../include/atlas/)
ENDIF ( OPT_ENABLE_ATLAS )

#handle GotoBLAS
OPTION(OPT_ENABLE_GOTOBLAS "Enable GOTOBLAS as Linear Algebra library" )
mark_as_advanced(
        OPT_ENABLE_GOTOBLAS
)

IF ( OPT_ENABLE_GOTOBLAS )
        SET(LINK_LIBRARIES
                ${LINK_LIBRARIES}


        )
        ADD_DEFINITIONS( -DSHARK_USE_GOTOBLAS )
        include_directories ( /include/ )
ENDIF ( OPT_ENABLE_GOTOBLAS )

#handle OpenMP
OPTION(OPT_ENABLE_OPENMP "Enable OpenMP" ON)
mark_as_advanced(
        OPT_ENABLE_OPENMP
)
IF( OPT_ENABLE_OPENMP )
        ADD_DEFINITIONS( -DSHARK_USE_OPENMP )
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ")
ENDIF( OPT_ENABLE_OPENMP )

#include directories
include_directories (
        /usr/local/include
)

# Get GCC version
if (CMAKE_COMPILER_IS_GNUCC)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                    OUTPUT_VARIABLE GCC_VERSION)
    string(REGEX MATCHALL "[0-9]+" GCC_VERSION_COMPONENTS ${GCC_VERSION})
    list(GET GCC_VERSION_COMPONENTS 0 GCC_MAJOR)
    list(GET GCC_VERSION_COMPONENTS 1 GCC_MINOR)

    message(STATUS "GCC version maj:min = " ${GCC_MAJOR} ":" ${GCC_MINOR})
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

set(JSONCPP_INCLUDE_DIR "${JSONCPP_INCLUDE_DIR}")

include_directories( "./include" )
#include_directories(${Boost_INCLUDE_DIRS})
include_directories(${JSONCPP_INCLUDE_DIR})
include_directories(SYSTEM ${JSONCPP_INCLUDE_DIR})

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Since I am using Boost, I need to do optimization
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)


ADD_EXECUTABLE(SarsaPolicy ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(SarsaPolicy ${YARP_LIBRARIES} ${JSONCPP_LIBRARY} ${LINK_LIBRARIES})
