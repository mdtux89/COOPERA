CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(SarsaLearner)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(JsonCpp REQUIRED)

file(GLOB folder_source ./src/*.cpp)
file(GLOB folder_header ./include/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

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
include_directories(${JSONCPP_INCLUDE_DIR})
include_directories(SYSTEM ${JSONCPP_INCLUDE_DIR})

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(SarsaLearner ${folder_source} ${folder_header})
message(STATUS ${JSONCPP_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(SarsaLearner ${YARP_LIBRARIES} ${JSONCPP_LIBRARY} ${LINK_LIBRARIES})
