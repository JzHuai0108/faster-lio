#include "laser_mapping_wrap.h"
#include "laser_mapping.h"
#include "utils.h"

namespace faster_lio {
bool LaserMappingWrap::InitROS(ros::NodeHandle &nh) {
    return laser_mapper_->InitROS(nh);
}

bool LaserMappingWrap::InitWithoutROS(const std::string &config_yaml) {
    return laser_mapper_->InitWithoutROS(config_yaml);
}

LaserMappingWrap::LaserMappingWrap() {
    laser_mapper_.reset(new LaserMapping());
}

LaserMappingWrap::~LaserMappingWrap() {
    laser_mapper_.reset();
}

void LaserMappingWrap::Run() {
    laser_mapper_->Run();
}

void LaserMappingWrap::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    laser_mapper_->StandardPCLCallBack(msg);
}

void LaserMappingWrap::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    laser_mapper_->LivoxPCLCallBack(msg);
}

void LaserMappingWrap::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    laser_mapper_->IMUCallBack(msg_in);
}

/////////////////////////////////////  debug save / show /////////////////////////////////////////////////////

void LaserMappingWrap::PublishPath(const ros::Publisher pub_path) {
    laser_mapper_->PublishPath(pub_path);
}

void LaserMappingWrap::PublishOdometry(const ros::Publisher &pub_odom_aft_mapped) {
    laser_mapper_->PublishOdometry(pub_odom_aft_mapped);
}

void LaserMappingWrap::PublishFrameWorld() {
    laser_mapper_->PublishFrameWorld();
}

void LaserMappingWrap::PublishFrameBody(const ros::Publisher &pub_laser_cloud_body) {
    laser_mapper_->PublishFrameBody(pub_laser_cloud_body);
}

void LaserMappingWrap::PublishFrameEffectWorld(const ros::Publisher &pub_laser_cloud_effect_world) {
    laser_mapper_->PublishFrameEffectWorld(pub_laser_cloud_effect_world);
}

void LaserMappingWrap::Savetrajectory(const std::string &traj_file, const common::V3D &I_p_B, const Eigen::Quaterniond &I_q_B) {
    laser_mapper_->Savetrajectory(traj_file, I_p_B, I_q_B);
}

void LaserMappingWrap::Finish(const std::string &output_dir) {
    laser_mapper_->Finish(output_dir);
}

std::string LaserMappingWrap::lid_topic() const {
    return laser_mapper_->lid_topic_;
}

std::string LaserMappingWrap::imu_topic() const {
    return laser_mapper_->imu_topic_;
}

Eigen::Vector3d LaserMappingWrap::I_p_B() const {
    return laser_mapper_->I_p_B_;
}

Eigen::Quaterniond LaserMappingWrap::I_q_B() const {
    return laser_mapper_->I_q_B_;
}
}  // namespace faster_lio