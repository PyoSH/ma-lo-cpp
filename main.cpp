#include "mapping/src/map_RT.h"

std::deque<Eigen::Matrix4f> vec_poses;
std::deque<double> vec_poses_time;
std::deque<Eigen::Matrix4Xf> vec_scan;
std::deque<double> vec_scan_time;

map_rt mapGenerator;
void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg);
void callback_pose(const nav_msgs::Odometry::ConstPtr& msg);
void check_data();

int main(int argc,char **argv) {
    ros::init(argc, argv, "ros_mapping");
    std::cout<<ros::this_node::getName()<<std::endl;
    ros::NodeHandle nh;

    ros::Subscriber sub_scan = nh.subscribe<sensor_msgs::PointCloud2>("/F/depth/color/points", 100, callback_scan);
    ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry>("/b1_controller/odom", 100, callback_pose);

    ros::spin();

    return 0;
}

/*
 * 현재) 발행 시간에 기반해서 데이터를 쳐낸다(시동기화)
 * 희망) 지도 작성에 사용된 이전 포즈 & 현재 포즈의 유클리드 거리로 거르는 조건도 추가하자.
 */
void check_data() {
    std::cout<<"check data init"<<std::endl;
    while(!vec_poses.empty() && !vec_scan.empty()) {
        if(fabs(vec_poses_time[0] - vec_scan_time[0]) > 0.1) {
            std::cout<<"check data - if case"<<std::endl;
            if(vec_poses_time[0] > vec_scan_time[0]) {
                vec_scan.pop_front();
                vec_scan_time.pop_front();
            }else {
                vec_poses.pop_front();
                vec_poses_time.pop_front();
            }
        }else {
            std::cout<<"check data - else case"<<std::endl;
            mapGenerator.updateMap(vec_poses[0], vec_scan[0]);
            vec_scan.pop_front();
            vec_scan_time.pop_front();
            vec_poses.pop_front();
            vec_poses_time.pop_front();
        }
    }
    std::cout<<"check data END"<<std::endl;
}

/*
 * 3D 점군 받았을 때 deque 구조로 넣을 수 있도록 하기
 */
void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::cout<<"CB - SCAN init"<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    int pointNum = pc_xyz->points.size();
    Eigen::Matrix4Xf eigenScan = Eigen::Matrix4Xf::Ones(4, 1);
    int usefulPoint = 0;
    std::cout<<"CB - SCAN 1"<<std::endl;
    for(int i = 0; i < pointNum; i++) {
        std::cout<<"CB - SCAN 2"<<std::endl;
        pcl::PointXYZ currPoint = pc_xyz->points[i];
        std::cout<<"CB - SCAN 3"<<std::endl;
        float dist = sqrt(pow(currPoint.x,2) + pow(currPoint.y,2) + pow(currPoint.z,2));
        if(0.2 < dist && dist < 5) {
            std::cout<<"CB - SCAN if case 1"<<std::endl;
            usefulPoint++;
            eigenScan.conservativeResize(4, usefulPoint);
            std::cout<<"CB - SCAN if case 2"<<std::endl;
            eigenScan(0,usefulPoint-1) = currPoint.x;
            eigenScan(1,usefulPoint-1) = currPoint.y;
            eigenScan(2,usefulPoint-1) = currPoint.z;
            eigenScan(3,usefulPoint-1) = 1;
            std::cout<<"CB - SCAN if case 3"<<std::endl;
        }
    }
    std::cout<<"CB - SCAN 4"<<std::endl;
    vec_scan.emplace_back(eigenScan);
    std::cout<<"CB - SCAN 5"<<std::endl;
    vec_scan_time.emplace_back(msg->header.stamp.toSec());
    check_data();
    std::cout<<"CB - SCAN END"<<std::endl;
}

void callback_pose(const nav_msgs::Odometry::ConstPtr& msg) {
    std::cout<<"CB - POSE init"<<std::endl;
    Eigen::Matrix4f eigenPose;

    tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    eigenPose << m[0][0], m[0][1], m[0][2], msg->pose.pose.position.x,
                 m[1][0], m[1][1], m[1][2], msg->pose.pose.position.y,
                 m[2][0], m[2][1], m[2][2], msg->pose.pose.position.z,
                 0, 0, 0, 1;
    vec_poses.emplace_back(eigenPose);
    vec_poses_time.emplace_back(msg->header.stamp.toSec());
    check_data();
    std::cout<<"CB - POSE END"<<std::endl;
}