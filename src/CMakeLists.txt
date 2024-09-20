add_library(${PROJECT_NAME}
        laser_mapping.cc
        laser_mapping_wrap.cc
        imu_processing.cc
        pointcloud_preprocess.cc
        options.cc
        use-ikfom.cc
        utils.cc
        ../include/glog/logging.cc
        )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp livox_ros_driver_gencpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        # tbb
        # glog
        yaml-cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})

install(TARGETS 
${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ../launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY ../include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
