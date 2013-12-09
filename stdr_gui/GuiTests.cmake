catkin_add_gtest(stdr_gui_laser_test
	tests/stdr_gui_laser_test.cpp 
)
target_link_libraries(stdr_gui_laser_test ${catkin_LIBRARIES} )
