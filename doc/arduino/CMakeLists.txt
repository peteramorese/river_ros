find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)


set(SOURCES
	src/Pixy_Repackage.cpp
	)

add_executable(Pixy_Pack src/Pixy_Repackage.cpp)
target_link_libraries(Pixy_Pack ${catkin_LIBRARIES})

##add_executable(rosserial src/serial_node.py)
##target_link_libraries(rosserial ${catkin_LIBRARIES})
install(TARGETS Pixy_Pack
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
