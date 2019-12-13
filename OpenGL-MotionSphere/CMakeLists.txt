cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

project  (Motion-Sphere)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set (SCRDIR "${PROJECT_SOURCE_DIR}/Source")
set (INCLUDEDIR "${PROJECT_SOURCE_DIR}/include")

#include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PROJECT_SOURCE_DIR}/lib)
#add_definitions     (${PCL_DEFINITIONS})
#include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

include_directories("${INCLUDEDIR}")
include_directories("${INCLUDEDIR}/Xsens")
include_directories("${INCLUDEDIR}/fgl")

SET  (project_SOURCES "${SCRDIR}/Main.cpp" "${SCRDIR}/XsensConnection.cpp" "${SCRDIR}/quaternion.cpp" "${SCRDIR}/Comparision.cpp" "${SCRDIR}/tgaload.cpp" "${SCRDIR}/SphereUtility.cpp")
SET  (project_HEADERS "${INCLUDEDIR}/XsensConnection.h" "${INCLUDEDIR}/quaternion.h" "${INCLUDEDIR}/Comparision.h" "${INCLUDEDIR}/tgaload.h" "${INCLUDEDIR}/SphereUtility.h")

add_executable(Motion-Sphere ${project_SOURCES} ${project_HEADERS} )
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Motion-Sphere)

target_link_libraries(Motion-Sphere freeglut.lib )
target_link_libraries(Motion-Sphere xsensdeviceapi64.lib )
target_link_libraries(Motion-Sphere xstypes64.lib)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Debug")
set(DLLDIRECTORY "${CMAKE_SOURCE_DIR}/bin") 
set(FILENAME "${CMAKE_CURRENT_BINARY_DIR}/Load" 
			 "${CMAKE_CURRENT_BINARY_DIR}/Load/Standard"
			 "${CMAKE_CURRENT_BINARY_DIR}/Load/Close"
			 "${CMAKE_CURRENT_BINARY_DIR}/Load/Wide"
)


file(GLOB DEPLOY_FILES_AND_DIRS "${PROJECT_SOURCE_DIR}/database/*")
foreach(ITEM ${DEPLOY_FILES_AND_DIRS})
   IF( IS_DIRECTORY "${ITEM}" )
      LIST( APPEND DIRS_TO_DEPLOY "${ITEM}" )
   ELSE()
      LIST( APPEND FILES_TO_DEPLOY "${ITEM}" )
   ENDIF()
endforeach()
INSTALL( FILES ${FILES_TO_DEPLOY} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Load )
INSTALL( DIRECTORY ${DIRS_TO_DEPLOY} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Load )

add_custom_command(
	TARGET Motion-Sphere
	POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory CData
	)

#foreach( file_i ${FILENAME})
#    add_custom_command(
#	TARGET Motion-Sphere
#	POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E make_directory ${file_i}
#	)
#endforeach( file_i )

add_custom_command(
    TARGET Motion-Sphere
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy "${CMAKE_SOURCE_DIR}/Texture/worldmgrs.tga" ${CMAKE_CURRENT_BINARY_DIR}
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
    TARGET Motion-Sphere
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${file_i} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
endforeach( file_i )
