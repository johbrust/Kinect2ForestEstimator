cmake_minimum_required(VERSION 3.0)

project(Kinect2ForestEstimator CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

MESSAGE( "CMAKE_SIZEOF_VOID_P: ${CMAKE_SIZEOF_VOID_P}" )

if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    MESSAGE( "64 bit compiler detected" )
    SET( EX_PLATFORM 64 )
    SET( EX_BIT_COUNT 64)
    SET( EX_PLATFORM_NAME "x64" )
else( CMAKE_SIZEOF_VOID_P EQUAL 8 ) 
    MESSAGE( "32 bit compiler detected" )
    SET( EX_PLATFORM 32 )
    SET( EX_BIT_COUNT 32)
    SET( EX_PLATFORM_NAME "x86" )
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

# set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
# find_package(Matlab REQUIRED)
MESSAGE (STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
find_package( OpenCV REQUIRED )
MESSAGE (STATUS "INC: ${OpenCV_INCLUDE_DIRS}")
MESSAGE (STATUS "LIB: ${OpenCV_LIBS}")

find_package( PCL 1.2 REQUIRED )
MESSAGE (STATUS "INC: ${PCL_INCLUDE_DIRS}")
MESSAGE (STATUS "LIB: ${PCL_LIBRARIES}")
MESSAGE (STATUS "LIBDIRS: ${PCL_LIBRARY_DIRS}")
MESSAGE (STATUS "DEF: ${PCL_DEFINITIONS}")

set(KINECTV2_INCLUDE_DIR $ENV{KINECTSDK20_DIR}inc)
set(KINECTV2_LIBRARIES 	$ENV{KINECTSDK20_DIR}Lib/${EX_PLATFORM_NAME}/Kinect20.lib
						$ENV{KINECTSDK20_DIR}Lib/${EX_PLATFORM_NAME}/Kinect20.Face.lib)

# include_directories(${KINECTV2_INCLUDE_DIR} ${MATLAB_ROOT}/toolbox/imaq/imaqadaptors/kit/include)
include_directories(${KINECTV2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# set(MATLAB_IMAQ_LIBRARY ${MATLAB_ROOT}/toolbox/imaq/imaqadaptors/kit/lib/win${EX_BIT_COUNT}/release/imaqmex.lib)

set(KV2B_SOURCE_DIR "src")

set(KV2B_SOURCE_FILES
		${KV2B_SOURCE_DIR}/main.cpp
		${KV2B_SOURCE_DIR}/Kinect2Grabber.h
		${KV2B_SOURCE_DIR}/Kinect2Grabber.cpp
		${KV2B_SOURCE_DIR}/CRForest.h
		${KV2B_SOURCE_DIR}/CRForestEstimator.h
		${KV2B_SOURCE_DIR}/CRForestEstimator.cpp
		${KV2B_SOURCE_DIR}/CRTree.h
		${KV2B_SOURCE_DIR}/CRTree.cpp)

add_executable(Kinect2ForestEstimator ${KV2B_SOURCE_FILES})
target_link_libraries(Kinect2ForestEstimator ${KINECTV2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_custom_command(TARGET Kinect2ForestEstimator POST_BUILD        						# Adds a post-build event to Kinect2ForestEstimator
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  								# which executes "cmake - E copy_if_different..."
        "$ENV{KINECTSDK20_DIR}Redist/Face/${EX_PLATFORM_NAME}/Kinect20.Face.dll"    # <--this is in-file
        $<TARGET_FILE_DIR:Kinect2ForestEstimator>                 						# <--this is out-file path
		
		COMMAND ${CMAKE_COMMAND} -E copy_directory  								# which executes "cmake - E copy_directory..."
        "$ENV{KINECTSDK20_DIR}Redist/Face/${EX_PLATFORM_NAME}/NuiDatabase"    		# <--this is in-dir
        $<TARGET_FILE_DIR:Kinect2ForestEstimator>/NuiDatabase							# <--this is out-dir path
		
		COMMAND ${CMAKE_COMMAND} -E copy_directory  								# which executes "cmake - E copy_directory..."
        "$ENV{PCL_ROOT}/3rdParty/VTK/bin"    										# <--this is in-dir
		$<TARGET_FILE_DIR:Kinect2ForestEstimator>)                 						# <--this is out-dir path