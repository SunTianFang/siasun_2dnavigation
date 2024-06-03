project( World )
cmake_minimum_required( VERSION 3.1 )
set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC ON )
set( CMAKE_BUILD_TYPE Release )
add_definitions ( -Wall )
find_package ( Qt5Widgets REQUIRED )
find_package ( Qt5Core REQUIRED )
find_package ( Qt5Gui REQUIRED )
set ( LIBRARY_OUTPUT_PATH $$LIFELONG_SLAM_DIR )
set ( World_HDRS
	
	)

set ( World_SRCS
	src/ArcPath.cpp
	src/GenericPath.cpp
	src/LinePath.cpp
	src/Node.cpp
	src/NodeBase.cpp
	src/Path.cpp
	src/PathBase.cpp
	src/RfId.cpp
	src/ScpPath.cpp
	src/SidePath.cpp
	src/UnknownPath.cpp
	src/Vertex.cpp
	src/World.cpp
	src/GraphicObj.cpp
	)

add_library ( World SHARED ${World_SRCS} ${UIS} ${RSCS} ${TRS} ${MOCS} )
target_link_libraries ( World  Qt5::Core Qt5::Gui Qt5::Widgets )
