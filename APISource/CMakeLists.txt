cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

project  (motion-matrix)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

#include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PROJECT_SOURCE_DIR}/lib)
#add_definitions     (${PCL_DEFINITIONS})
#include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_BINARY_DIR}")


include_directories("${PROJECT_SOURCE_DIR}/include/Xsens")
include_directories("${PROJECT_SOURCE_DIR}/include/fgl")

SET  (project_SOURCES Robot_Xsens.cpp XsensConnection.cpp quaternion.cpp Comparision.cpp tgaload.cpp)
SET  (project_HEADERS XsensConnection.h quaternion.h Comparision.h tgaload.h)

add_executable(motion-matrix ${project_SOURCES} ${project_HEADERS} )
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT motion-matrix)

target_link_libraries(motion-matrix freeglut.lib )
target_link_libraries(motion-matrix xsensdeviceapi64.lib )
target_link_libraries(motion-matrix xstypes64.lib)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Debug")
set(DLLDIRECTORY "${CMAKE_SOURCE_DIR}/bin") 
set(FILENAME "${CMAKE_CURRENT_BINARY_DIR}/Load" 
			 "${CMAKE_CURRENT_BINARY_DIR}/Load/Standard"
			 "${CMAKE_CURRENT_BINARY_DIR}/Load/Close"
			 "${CMAKE_CURRENT_BINARY_DIR}/Load/Wide"
)


foreach( file_i ${FILENAME})
    add_custom_command(
	TARGET motion-matrix
	POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${file_i}
	)
endforeach( file_i )

add_custom_command(
    TARGET motion-matrix
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy "${CMAKE_SOURCE_DIR}/worldmgrs.tga" ${CMAKE_CURRENT_BINARY_DIR}
	)
# create a list of files to copy
set( THIRD_PARTY_DLLS
   "${DLLDIRECTORY}/freeglut.dll" 
   "${DLLDIRECTORY}/xsensdeviceapi_com64.dll" 
   "${DLLDIRECTORY}/xsensdeviceapi64.dll" 
   "${DLLDIRECTORY}/xstypes64.dll"
   )
   

# do the copying
foreach( file_i ${THIRD_PARTY_DLLS})
    add_custom_command(
    TARGET motion-matrix
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${file_i} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
endforeach( file_i )
