add_executable(manipulator_node src/manipulator_node.cpp)

#add_subdirectory(include)
#target_include_directories(${PROJECT_NAME} PUBLIC include)
#link_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(manipulator_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

