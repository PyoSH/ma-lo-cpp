#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <pcl/point_types.h>  // 포인트 타입 정의
#include <pcl/point_cloud.h>  // 포인트 클라우드 클래스
#include <pcl_conversions/pcl_conversions.h>  // ros msg -> point cloud
#include <pcl/filters/passthrough.h>  // 범위 필터
#include <pcl/filters/voxel_grid.h>  // 다운샘플링 필터
#include <pcl/filters/passthrough.h>  // 범위 필터
#include "tool.h"

std::deque<Eigen::Matrix4Xf> vec_scan_F;
std::deque<double> vec_scan_F_time;
std::deque<Eigen::Matrix4Xf> vec_scan_R;
std::deque<double> vec_scan_R_time;
bool isPubOngoing = false;

void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& topic_name);
void merge_AND_pub(Eigen::Matrix4Xf scan1, Eigen::Matrix4Xf scan2, double t1, double t2);
void check_data();

int main(int argc,char **argv) {
    ros::init(argc, argv, "merge_PCs");
    std::cout<<ros::this_node::getName()<<std::endl;
    ros::NodeHandle nh;

    // ros::Subscriber sub_scan_F = nh.subscribe<sensor_msgs::PointCloud2>("/F/depth/color/points", 100, callback_scan);
    // ros::Subscriber sub_scan_R = nh.subscribe<sensor_msgs::PointCloud2>("/R/depth/color/points", 100, callback_scan);
    ros::Subscriber sub_scan_F = nh.subscribe<sensor_msgs::PointCloud2>(
        "/F/depth/color/points", 100, 
        [&](const sensor_msgs::PointCloud2::ConstPtr& msg) { callback_scan(msg, "/F/depth/color/points"); });

    ros::Subscriber sub_scan_R = nh.subscribe<sensor_msgs::PointCloud2>(
        "/R/depth/color/points", 100, 
        [&](const sensor_msgs::PointCloud2::ConstPtr& msg) { callback_scan(msg, "/R/depth/color/points"); });

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), [](const ros::TimerEvent&) {check_data();});
    ros::spin();

    return 0;
}

/*
 * 3D 점군 받았을 때 deque 구조로 넣을 수 있도록 하기
 */
void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& topic_name) {
    std::cout<<"CB - SCAN init"<< topic_name <<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc_xyz);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_vx_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(pc_xyz);
    voxel_filter.setLeafSize(0.2, 0.2, 0.2); // 0.1
    voxel_filter.filter(*pc_vx_filtered);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pc_vx_filtered);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(-2.0, 2.0);  
     pass.setFilterFieldName("y");
    pass.setFilterLimits(0.2, 1.0);  
     pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1, 6.0); 
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*pc_filtered);

    int pointNum = pc_filtered->points.size();
    Eigen::Matrix4Xf eigenScan = Eigen::Matrix4Xf::Ones(4, 1);
    int usefulPoint = 0;
    std::cout<<"CB - pointNum "<< pointNum <<std::endl;

    for(int i = 0; i < pointNum; i++) {
        // std::cout<<"CB - SCAN 2"<<std::endl;
        pcl::PointXYZ currPoint = pc_filtered->points[i];
        // std::cout<<"CB - SCAN 3"<<std::endl;
        float dist = sqrt(pow(currPoint.x,2) + pow(currPoint.y,2) + pow(currPoint.z,2));
        if(0.2 < dist && dist < 4) {
            usefulPoint++;
            eigenScan.conservativeResize(4, usefulPoint);
            eigenScan(0,usefulPoint-1) = currPoint.x;
            eigenScan(1,usefulPoint-1) = currPoint.y;
            eigenScan(2,usefulPoint-1) = currPoint.z;
            eigenScan(3,usefulPoint-1) = 1;
        }
    }

    if (topic_name == "/F/depth/color/points") {
        vec_scan_F.emplace_back(eigenScan);
        vec_scan_F_time.emplace_back(msg->header.stamp.toSec());
    } else if (topic_name == "/R/depth/color/points") {
        vec_scan_R.emplace_back(eigenScan);
        vec_scan_R_time.emplace_back(msg->header.stamp.toSec());
    }
    check_data();
    // std::cout<<"CB - SCAN END"<<std::endl;
}

void check_data() {
    // std::cout<<"check data init"<<std::endl;
    while(!vec_scan_F.empty() && !vec_scan_R.empty()) {
        // std::cout<<"check data - if case"<<std::endl;
        std::cout<<"scan_F t :" <<vec_scan_F_time[0] << std::setprecision(9)<<
            "||" <<"SCAN_R t :" <<vec_scan_R_time[0] << std::setprecision(9) <<std::endl;
        if(fabs(vec_scan_F_time[0] - vec_scan_R_time[0]) > 0.1) {
            if(vec_scan_F_time[0] > vec_scan_R_time[0]) {
                vec_scan_R.pop_front();
                vec_scan_R_time.pop_front();
            }else {
                vec_scan_F.pop_front();
                vec_scan_F_time.pop_front();
            }
        }else {
            // std::cout<<"check data - else case"<<std::endl;
            merge_AND_pub(vec_scan_F[0], vec_scan_R[0], vec_scan_F_time[0],  vec_scan_R_time[0]);
            vec_scan_F.pop_front();
            vec_scan_F_time.pop_front();
            vec_scan_R.pop_front();
            vec_scan_R_time.pop_front();            
        }
    }
    // std::cout<<"check data END"<<std::endl;
}

void merge_AND_pub(Eigen::Matrix4Xf scan1, Eigen::Matrix4Xf scan2, double t1, double t2){
    // transform matrix : 2nd sensor frame to 1st sensor frame
    Eigen::Matrix4f tf_2to1;
    float roll_rad = 0;
    float pitch_rad = 90 * M_PI / 180.0;
    float yaw_rad = 0;
    Eigen::Matrix3f rotation = tool::get_rotation_matrix(roll_rad, pitch_rad, yaw_rad);
    tf_2to1.block<3,3>(0,0) << rotation;
    tf_2to1(0,3) = 0.2;
    tf_2to1(1,3) = 0;
    tf_2to1(2,3) = -0.3;
    tf_2to1(3,3) = 1;

    // apply tf to scan2
    Eigen::Matrix4Xf transformed_scan2 = tf_2to1 * scan2;
    
    // merge PCs
    Eigen::Matrix4Xf merged_scan(4, scan1.cols() + transformed_scan2.cols());
    merged_scan << scan1, transformed_scan2;
    pcl::PointCloud<pcl::PointXYZ> merged_PC_pcl;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr merged_PC_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<merged_scan.cols(); ++i){
        pcl::PointXYZ currPoint;
        currPoint.x = merged_scan(0,i);
        currPoint.y = merged_scan(1,i);
        currPoint.z = merged_scan(2,i);
        merged_PC_pcl.push_back(currPoint);
    }


    // pack into ROS msg with avg time 
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(merged_PC_pcl, output_msg);
    output_msg.header.frame_id = "odom";
    double avg_time = (t1 + t2) / 2.0;
    output_msg.header.stamp = ros::Time(avg_time);
    // output_msg.header.stamp = ros::Time::now();
    
    // pub them
    static ros::Publisher pub_mergedPC = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/merged_pointcloud", 100);
    pub_mergedPC.publish(output_msg);
}

Eigen::Matrix3f get_rotation_matrix(float roll, float pitch, float yaw) {
    // Rx: roll에 대한 회전 행렬
    Eigen::Matrix3f Rx;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

    // Ry: pitch에 대한 회전 행렬
    Eigen::Matrix3f Ry;
    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);

    // Rz: yaw에 대한 회전 행렬
    Eigen::Matrix3f Rz;
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;

    // 전체 회전 행렬: Rz * Ry * Rx
    Eigen::Matrix3f rotation_matrix = Rz * Ry * Rx;

    return rotation_matrix;
}
