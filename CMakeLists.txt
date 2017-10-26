cmake_minimum_required(VERSION 2.8)
project(DLite)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra")
set(CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++")
set(SOURCE_FILES
	main.cpp
    	config.cpp
    	environmentoptions.cpp
    	map.cpp
    	mission.cpp
   	tinyxml2.cpp
    	xmllogger.cpp
    	dlite.cpp
    	localmap.cpp)

set(HEADER_FILES
	config.h
    	environmentoptions.h
    	gl_const.h
    	heuristics.h
   	ilogger.h
    	map.h
    	mission.h
    	node.h
    	openlist.h
    	searchresult.h
    	tinyxml2.h
    	xmllogger.h
    	dlite.h
    	localmap.h)
add_executable(LPAstar ${SOURCE_FILES} ${HEADER_FILES})
