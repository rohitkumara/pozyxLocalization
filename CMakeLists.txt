cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project( pozyx_ros )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories( ${PROJECT_SOURCE_DIR}/include/ )
include_directories( ${PROJECT_SOURCE_DIR}/src/ )

SET( SOURCES ${PROJECT_SOURCE_DIR}/src/pozyx_kalman.cpp
	    ${PROJECT_SOURCE_DIR}/src/serial_lib.c )

SET( HEADERS ${PROJECT_SOURCE_DIR}/include/serial_lib.h ) 

add_executable( demo ${SOURCES} ${HEADERS})

set_target_properties( demo	
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib"
	LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib"
	RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

target_link_libraries( demo ${OpenCV_LIBS})
