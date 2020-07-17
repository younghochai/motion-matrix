cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

# Require C++11
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

#OpenIA(InterAction, Intelligence Amplification) OSS
project  (OpenIA)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set (SCRDIR "${PROJECT_SOURCE_DIR}/Source")
set (INCLUDEDIR "${PROJECT_SOURCE_DIR}/include")


#include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PROJECT_SOURCE_DIR}/Lib)
#add_definitions     (${PCL_DEFINITIONS})
#include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

include_directories("${INCLUDEDIR}")
include_directories("${INCLUDEDIR}/Xsens")
include_directories("${INCLUDEDIR}/fgl")
include_directories("${SCRDIR}/externLib")


SET  (project_SOURCES "${SCRDIR}/iaMain.cpp" "${SCRDIR}/iaQuaternion.cpp" "${SCRDIR}/iaAcquireGesture.cpp" "${SCRDIR}/iaPositionTracking.cpp" "${SCRDIR}/iaSphereUtility.cpp" "${SCRDIR}/iaVitruvianAvatar.cpp" "${SCRDIR}/iaMotionSphere.cpp" "${SCRDIR}/iaMotionAuthoring.cpp" "${SCRDIR}/externLib/XsensConnection.cpp" "${SCRDIR}/externLib/tgaload.cpp" "${SCRDIR}/externLib/Model_PLY.cpp")
SET  (project_HEADERS "${INCLUDEDIR}/iaQuaternion.h" "${INCLUDEDIR}/iaAcquireGesture.h" "${INCLUDEDIR}/iaPositionTracking.h" "${INCLUDEDIR}/iaSphereUtility.h" "${INCLUDEDIR}/iaVitruvianAvatar.h" "${INCLUDEDIR}/iaMotionAuthoring.h" "${INCLUDEDIR}/iaMotionSphere.h" "${SCRDIR}/externLib/tgaload.h" "${SCRDIR}/externLib/Model_PLY.h" "${SCRDIR}/externLib/VelodyneCapture.h")

add_executable(OpenIA ${project_SOURCES} ${project_HEADERS} )
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT OpenIA)

target_link_libraries(OpenIA freeglut.lib )
target_link_libraries(OpenIA xsensdeviceapi64.lib )
target_link_libraries(OpenIA xstypes64.lib)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Debug")
set(DLLDIRECTORY "${CMAKE_SOURCE_DIR}/bin") 
set(FILENAME "${CMAKE_CURRENT_BINARY_DIR}/data" 
			 "${CMAKE_CURRENT_BINARY_DIR}/data/Rotationdata"			
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
	TARGET OpenIA
	POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory CData
	)
	
add_custom_command(
	TARGET OpenIA
	POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory LiDARData
	)
	
add_custom_command(
	TARGET OpenIA
	POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory PositionData
	)
	
add_custom_command(
	TARGET OpenIA
	POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory SkeletonData
	)
add_custom_command(
	TARGET OpenIA
	POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory Target
	)
	
add_custom_command(
	TARGET OpenIA
	POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory TotalCapture
	)


#foreach( file_i ${FILENAME})
#    add_custom_command(
#	TARGET OpenIA
#	POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E make_directory ${file_i}
#	)
#endforeach( file_i )

add_custom_command(
    TARGET OpenIA
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
    TARGET OpenIA
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy ${file_i} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
endforeach( file_i )

#Posiont tracking related
# Find Packages
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries (OpenIA ${PCL_LIBRARIES})

# Find Package Thread
find_package( Threads REQUIRED )

# Find Package Boost
set( BOOST_DIR "C:/Program Files/PCL 1.9.0/3rdParty/Boost" )
set( Boost_USE_STATIC_LIBS ON ) # Static Link Libraries ( libboost_* )
set( Boost_USE_MULTITHREADED ON ) # Multi Thread Libraries ( *-mt-* )
set( Boost_USE_STATIC_RUNTIME OFF ) # Static Runtime Libraries ( *-s* )
find_package( BOOST COMPONENTS system date_time regex )

# If Direct Capture from Sensors, VelodyneCapture are required Boost.Asio.
# Please Define HAVE_BOOST in Preprocessor.
set( HAVE_BOOST )
if( Boost_FOUND )
  set( HAVE_BOOST "-DHAVE_BOOST" )
endif()

# Find Package PCAP
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
set( PCAP_DIR "C:/WpdPack" )
find_package( PCAP )

# If Capture from PCAP Files, VelodyneCapture are required PCAP.
# Please Define HAVE_PCAP in Preprocessor.
set( HAVE_PCAP )
if( PCAP_FOUND )
  set( HAVE_PCAP "-DHAVE_PCAP" )
endif()

# Set Properties
if( PCAP_FOUND OR Boost_FOUND )
  # Additional Include Directories
  include_directories( ${Boost_INCLUDE_DIRS} )
  include_directories( ${PCAP_INCLUDE_DIRS} )

  # Add Definitions
  add_definitions( ${Boost_LIB_DIAGNOSTIC_DEFINITIONS} )
  add_definitions( ${HAVE_BOOST} )
  add_definitions( ${HAVE_PCAP} )

  # Additional Library Directories
  link_directories( ${Boost_LIBRARY_DIRS} )
  link_directories( ${PCAP_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( OpenIA ${CMAKE_THREAD_LIBS_INIT} )
  target_link_libraries( OpenIA ${Boost_LIBRARIES} )
  target_link_libraries( OpenIA ${PCAP_LIBRARIES} )
else()
  message( WARNING "VelodyneCapture need at least either Boost or PCAP." )
endif()