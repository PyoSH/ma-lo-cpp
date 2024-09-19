#include "mapping/src/map_RT.h"

std::deque<Eigen::Matrix4f> vec_poses;
std::deque<double> vec_poses_time;
std::deque<Eigen::Matrix4f> vec_scan;
std::deque<double> vec_scan_time;

map_rt mapGenerator;
void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg);
void callback_pose(const nav_msgs::Odometry::ConstPtr& msg);
void check_data();

int main(int argc,char **argv) {
    ros::init(argc, argv, "ros_mapping");
    std::cout<<ros::this_node::getName()<<std::endl;
    ros::NodeHandle nh;

    ros::Subscriber sub_scan = nh.subscribe<sensor_msgs::PointCloud2>("/scan", 100, callback_scan);
    ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry>("/b1_controller/odom", 100, callback_pose);

    ros::spin();

    return 0;
}

void check_data() {
    while((vec_poses_time.size() != 0 && vec_scan.size() != 0)) {
        if(fabs(vec_poses_time[0] - vec_scan_time[0]) > 0.1) {
            if(vec_poses_time[0] > vec_scan_time[0]) {
                vec_scan.pop_front();
                vec_scan_time.pop_front();
            }else {
                vec_poses.pop_front();
                vec_poses_time.pop_front();
            }
        }else {
            mapGenerator.updateMap(vec_poses[0], vec_scan[0]);
            vec_scan.pop_front();
            vec_scan_time.pop_front();
            vec_poses.pop_front();
            vec_poses_time.pop_front();
        }
    }
}

void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // int pointNum = msg.data



    check_data();
}


void callback_pose(const nav_msgs::Odometry::ConstPtr& msg) {
    Eigen::Matrix4f eigenPose;

    tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    eigenPose << m[0][0], m[0][1], m[0][2], msg->pose.pose.position.x,
                 m[1][0], m[1][1], m[1][2], msg->pose.pose.position.y,
                 m[2][0], m[2][1], m[2][2], msg->pose.pose.position.z,
                 0, 0, 0, 1;
    vec_poses.push_back(eigenPose);
    vec_poses_time.push_back(msg->header.stamp.toSec());
    check_data();
}