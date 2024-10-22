#ifndef FASTER_LIO_LASER_MAPPING_WRAP_H
#define FASTER_LIO_LASER_MAPPING_WRAP_H

#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <condition_variable>
#include <thread>

#include <Eigen/Geometry>

#include "options.h"

namespace faster_lio {
class LaserMapping;

class LaserMappingWrap {
   public:
    LaserMappingWrap();
    ~LaserMappingWrap();

    /// init with ros
    bool InitROS(ros::NodeHandle &nh);

    /// init without ros
    bool InitWithoutROS(const std::string &config_yaml);

    void Run();

    // callbacks of lidar and imu
    void StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in);

    void PublishPath(const ros::Publisher pub_path);
    void PublishOdometry(const ros::Publisher &pub_odom_aft_mapped);
    void PublishFrameWorld();
    void PublishFrameBody(const ros::Publisher &pub_laser_cloud_body);
    void PublishFrameEffectWorld(const ros::Publisher &pub_laser_cloud_effect_world);
    void Savetrajectory(const std::string &traj_file, const Eigen::Vector3d &I_p_B, const Eigen::Quaterniond &I_q_B);

    void Finish(const std::string &output_dir);

    std::string lid_topic() const;
    std::string imu_topic() const;
    Eigen::Vector3d I_p_B() const;
    Eigen::Quaterniond I_q_B() const;

   private:
    std::shared_ptr<LaserMapping> laser_mapper_;
};

}  // namespace faster_lio

#endif  // FASTER_LIO_LASER_MAPPING_WRAP_H