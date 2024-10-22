//
// Created by xiang on 2021/10/8.
//
#include <unistd.h>
#include <csignal>

#include <glog/logging.h>

#include "laser_mapping_wrap.h"
#include "utils.h"

/// run the lidar mapping in online mode

// DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    // FLAGS_stderrthreshold = google::INFO;
    // FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    // google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "faster_lio");

    std::string traj_log_file = "./Log/traj.txt";
    if (argc >= 2) {
        traj_log_file = argv[1];
    }
    std::cout << "Traj log file " << traj_log_file << std::endl;

    ros::NodeHandle nh;

    auto laser_mapping = std::make_shared<faster_lio::LaserMappingWrap>();
    laser_mapping->InitROS(nh);

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);

    // online, almost same with offline, just receive the messages from ros
    while (ros::ok()) {
        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
        ros::spinOnce();
        laser_mapping->Run();
        rate.sleep();
    }

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish("");

    faster_lio::Timer::PrintAll();
    LOG(INFO) << "save trajectory to: " << traj_log_file;
    laser_mapping->Savetrajectory(traj_log_file, laser_mapping->I_p_B(), laser_mapping->I_q_B());

    return 0;
}
