#############################################################################
#############################################################################
#
#  This is a CMakeList.txt file originally programmed for the CLMC/AMD labs
#  at the University of Southern California and the Max-Planck-Institute for
#  Intelligent Systems. We use a mixutre of explicit makefiles and cmake, but 
#  primarily we relay on cmake for all major compile dependencies. All our
#  software is provided under a slightly modified version of the LGPL license
#  to be found at http://www-clmc.usc.edu/software/license.
#
#  Copyright by Stefan Schaal, 2014
#
#############################################################################
#############################################################################
# which version are we using

cmake_minimum_required(VERSION 2.8)

#############################################################################
# include common cmake components

include($ENV{LAB_ROOT}/config/cmake/LAB.cmake)

#############################################################################
# user defined cmake components

# set global compile type
set(CMAKE_BUILD_TYPE RelWithDebInfo) # Optimization with debugging info
#set(CMAKE_BUILD_TYPE Release)       # Optimization 
#set(CMAKE_BUILD_TYPE Debug)         # Debug

# local defines
include_directories(BEFORE ../src)
include_directories(BEFORE ../include)

set(SOURCES
	robotiq_2f_gripper.cc
	robotiq_2f_gripper_serial.cc	
	)

set(HEADERS
	../include/robotiq_2f_gripper.h
	../include/robotiq_2f_gripper_serial.h	
	)

add_library(grippers ${SOURCES})
install(FILES ${HEADERS} DESTINATION ${LAB_INCLUDES})
install(TARGETS grippers ARCHIVE DESTINATION ${LAB_LIBDIR})

add_executable(xr2f robotiq_2f_gripper_test.cc)
target_link_libraries(xr2f grippers comm ${LAB_STD_LIBS})
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/xr2f DESTINATION ${LAB_BINDIR})

add_executable(xr2fs robotiq_2f_gripper_serial_test.cc)
target_link_libraries(xr2fs grippers comm ${LAB_STD_LIBS})
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/xr2fs DESTINATION ${LAB_BINDIR})
