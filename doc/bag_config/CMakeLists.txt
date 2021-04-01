find_package(
	Armadillo REQUIRED
	PythonInterp
	PythonLibs
	)

include_directories(
	${ARMADILLO_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIR}
	headers
	matplotlib-cpp-master
	)

set(SOURCES
	src/main.cpp
	src/BAG.cpp
	src/DATA.cpp
	src/MARKER.cpp
	src/SENSOR.cpp
	src/SENSOR_MIN.cpp
	src/SYSTEM.cpp
	src/matrix.cpp
	src/oriDetermine.cpp
	)

set(TALKER_SOURCES
	src/talker.cpp
	)

add_executable(position
	${SOURCES}
	)
add_executable(talker_ros
	${TALKER_SOURCES}
	)

target_link_libraries(position 
	${ARMADILLO_LIBRARIES}
	${PYTHON_LIBRARY}
	${catkin_LIBRARIES}
	)

target_link_libraries(talker_ros
	${catkin_LIBRARIES}
	)