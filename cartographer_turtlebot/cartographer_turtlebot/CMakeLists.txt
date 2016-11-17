google_test(configuration_files_test
  USES_CARTOGRAPHER
  SRCS
    configuration_files_test.cc
)

google_binary(cartographer_flat_world_imu_node
  SRCS
    flat_world_imu_node_main.cc
)

install(TARGETS cartographer_flat_world_imu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
