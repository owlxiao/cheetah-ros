add_executable(imu-test 
    ImuTest.cpp 
)

target_link_libraries(imu-test 
   legged_cheetah_sdk 
)

install(TARGETS imu-test 
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)