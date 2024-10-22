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

void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg);
void pubThis(Eigen::Matrix4Xf scan, double t1);

int main(int argc,char **argv) {
    ros::init(argc, argv, "groundRemoval");
    std::cout<<ros::this_node::getName()<<std::endl;
    ros::NodeHandle nh;

    ros::Subscriber sub_scan_F = nh.subscribe<sensor_msgs::PointCloud2>("/merged_pointcloud", 100, callback_scan);
    
    ros::spin();

    return 0;
}

/*
 * 3D 점군 받았을 때 deque 구조로 넣을 수 있도록 하기
 */
void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::cout<<"CB - SCAN init"<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc_xyz);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_noGround(new pcl::PointCloud<pcl::PointXYZ>);
    // tool::removeGroundPlane(pc_xyz, pc_noGround);
    tool::removeGroundPlaneWithNormal(pc_xyz, pc_noGround, 0.1, 0.1);

    int pointNum = pc_noGround->points.size();
    Eigen::Matrix4Xf eigenScan = Eigen::Matrix4Xf::Ones(4, 1);
    std::cout<<"CB - pointNum "<< pointNum <<std::endl;

    for(int i = 1; i < pointNum+1; i++) {
        pcl::PointXYZ currPoint = pc_noGround->points[i-1];
        
        eigenScan.conservativeResize(4, i);
        eigenScan(0,i-1) = currPoint.x;
        eigenScan(1,i-1) = currPoint.y;
        eigenScan(2,i-1) = currPoint.z;
        eigenScan(3,i-1) = 1;
    }

    pubThis(eigenScan, (double)msg->header.stamp.toSec());
    std::cout<<"CB - SCAN END"<<std::endl;
}

void pubThis(Eigen::Matrix4Xf scan, double t1){
    std::cout<<"PUB - init"<<std::endl;
    // transform matrix : camera frame to odom(world) frame
    Eigen::Matrix4f tf_camToOdom;
    float roll_rad = -90* M_PI / 180.0;
    float pitch_rad = 0;
    float yaw_rad = -90* M_PI / 180.0;
    Eigen::Matrix3f rotation = tool::get_rotation_matrix(roll_rad, pitch_rad, yaw_rad);
    tf_camToOdom.block<3,3>(0,0) << rotation;
    tf_camToOdom(0,3) = 0;
    tf_camToOdom(1,3) = 0;
    tf_camToOdom(2,3) = 0;
    tf_camToOdom(3,3) = 1;

    // apply tf to scan (cam -> odom)
    Eigen::Matrix4Xf transformed_scan = tf_camToOdom * scan;
    
    // eigen -> PCL vector 
    pcl::PointCloud<pcl::PointXYZ> removed_PC_pcl;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr merged_PC_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<transformed_scan.cols(); ++i){
        pcl::PointXYZ currPoint;
        currPoint.x = transformed_scan(0,i);
        currPoint.y = transformed_scan(1,i);
        currPoint.z = transformed_scan(2,i);
        removed_PC_pcl.push_back(currPoint);
    }

    // pack into ROS msg
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(removed_PC_pcl, output_msg);
    output_msg.header.frame_id = "odom";
    // output_msg.header.stamp = ros::Time(t1);
    output_msg.header.stamp = ros::Time::now();
    
    // pub them
    static ros::Publisher pub_mergedPC = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/groundRemoved_pointcloud", 100);
    pub_mergedPC.publish(output_msg);
    std::cout<<"PUB - END"<<std::endl;
}