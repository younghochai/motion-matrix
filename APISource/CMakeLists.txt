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

target_link_libraries(motion-matrix freeglut.lib )
target_link_libraries(motion-matrix xsensdeviceapi64.lib )
target_link_libraries(motion-matrix xstypes64.lib)





