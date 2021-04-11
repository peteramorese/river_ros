add_executable(task_planner_node src/task_planner_node.cpp)
#add_executable(eef_srv_test_node src/eef_srv_test_node.cpp)

add_subdirectory(include)
#target_include_directories(task_planner_node PUBLIC include)
link_directories(task_planner_node PUBLIC include)
#link_directories(eef_srv_test_node PUBLIC include)
target_link_libraries(task_planner_node 
	EdgeClass
	AstarClass
	StateClass
	ConditionClass
	TransitionSystemClass
	${catkin_LIBRARIES}
	)

#target_link_libraries(eef_srv_test_node ${catkin_LIBRARIES})

install(TARGETS task_planner_node 
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

