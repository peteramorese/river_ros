add_executable(bag_config_test_node src/bag_config_test_node.cpp)

#add_subdirectory(include)
#target_include_directories(${PROJECT_NAME} PUBLIC include)
#link_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(bag_config_test_node ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

