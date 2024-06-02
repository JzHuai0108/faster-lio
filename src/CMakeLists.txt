add_library(${PROJECT_NAME}
        laser_mapping.cc
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
