add_executable(status_node src/status_node.cpp)

#add_subdirectory(include)
#target_include_directories(task_planner_node PUBLIC include)
#link_directories(task_planner_node PUBLIC include)
target_link_libraries(status_node 
	EdgeClass
	#AstarClass
	StateClass
	ConditionClass
	#TransitionSystemClass
	${catkin_LIBRARIES}
	)

target_link_libraries(status_node ${catkin_LIBRARIES})

install(TARGETS status_node 
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

