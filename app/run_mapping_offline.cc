//
// Created by xiang on 2021/10/9.
//

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unistd.h>
#include <csignal>

#include <glog/logging.h>

#include "laser_mapping_wrap.h"
#include "utils.h"

/// run faster-LIO in offline mode

// DEFINE_string(config_file, "./config/avia.yaml", "path to config file");
// DEFINE_string(bag_file, "/home/xiang/Data/dataset/fast_lio2/avia/2020-09-16-quick-shack.bag", "path to the ros bag");
// DEFINE_string(time_log_file, "./Log/time.log", "path to time log file");
// DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    // gflags::ParseCommandLineFlags(&argc, &argv, true);
    // FLAGS_stderrthreshold = google::INFO;
    // FLAGS_colorlogtostderr = true;

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <CONFIG_FILE> <BAG_FILE> [TIME_LOG_FILE=Log/time.log] [TRAJ_LOG_FILE=Log/traj.txt]"
                  << std::endl;
        return -1;
    }
    std::string FLAGS_config_file = argv[1];
    std::string FLAGS_bag_file = argv[2];
    std::string FLAGS_time_log_file = "./Log/time.log";
    if (argc >= 4) {
        FLAGS_time_log_file = argv[3];
    }
    std::string traj_log_file = "./Log/traj.txt";
    if (argc >= 5) {
        traj_log_file = argv[4];
    }

    google::InitGoogleLogging(argv[0]);

    const std::string bag_file = FLAGS_bag_file;
    const std::string config_file = FLAGS_config_file;

    auto laser_mapping = std::make_shared<faster_lio::LaserMappingWrap>();
    if (!laser_mapping->InitWithoutROS(FLAGS_config_file)) {
        LOG(ERROR) << "laser mapping init failed.";
        return -1;
    }

    /// handle ctrl-c
    signal(SIGINT, SigHandle);

    // just read the bag and send the data
    LOG(INFO) << "Opening rosbag, be patient";
    rosbag::Bag bag(FLAGS_bag_file, rosbag::bagmode::Read);

    LOG(INFO) << "Go!";

    std::vector<std::string> topics;
    topics.push_back(laser_mapping->lid_topic());
    topics.push_back(laser_mapping->imu_topic());
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (const rosbag::MessageInstance &m : view) {
        auto livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_msg) {
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &livox_msg]() {
                    laser_mapping->LivoxPCLCallBack(livox_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
            continue;
        }

        auto point_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (point_cloud_msg) {
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &point_cloud_msg]() {
                    laser_mapping->StandardPCLCallBack(point_cloud_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
            continue;
        }

        auto imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg) {
            laser_mapping->IMUCallBack(imu_msg);
            continue;
        }

        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
    }

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish("");

    /// print the fps
    double fps = 1.0 / (faster_lio::Timer::GetMeanTime("Laser Mapping Single Run") / 1000.);
    LOG(INFO) << "Faster LIO average FPS: " << fps;

    LOG(INFO) << "save trajectory to: " << traj_log_file;

    laser_mapping->Savetrajectory(traj_log_file, laser_mapping->I_p_B(), laser_mapping->I_q_B());

    faster_lio::Timer::PrintAll();
    faster_lio::Timer::DumpIntoFile(FLAGS_time_log_file);

    return 0;
}
