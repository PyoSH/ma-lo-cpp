#include "src/mcl.h"

// std::deque<Eigen::Matrix4f> vec_poses;
// std::deque<double> vec_poses_time;
// std::deque<Eigen::Matrix4Xf> vec_scan;
// std::deque<double>vec_scan_time;
std::vector<Eigen::Matrix4f> vec_poses;
std::vector<double> vec_poses_time;
std::vector<Eigen::Matrix4Xf> vec_scan;
std::vector<double>vec_scan_time;

mcl mclocalizer;
void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg);
void callback_pose(const nav_msgs::Odometry::ConstPtr& msg);
void check_data();

int main(int argc, char **argv){
    ros::init(argc, argv, "ros_mcl_test");
    std::cout<< ros::this_node::getName() << std::endl;

    ros::NodeHandle nh;
    ros::Subscriber sub_scan = nh.subscribe<sensor_msgs::PointCloud2>("/merged_pointcloud",100,callback_scan);
    ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry>("/b1_controller/odom",100,callback_pose);
    
    ros::spin();

    return 0;
}

void check_data(){
  while(!vec_poses.empty() && !vec_scan.empty())
  {
    std::cout<<"CHECK-DATA init"<<std::endl;
    if(fabs(vec_poses_time[0] - vec_scan_time[0])>0.1){
      if(vec_poses_time[0]>vec_scan_time[0]){
        vec_scan.erase(vec_scan.begin());
        vec_scan_time.erase(vec_scan_time.begin());
      }else{
        vec_poses.erase(vec_poses.begin());
        vec_poses_time.erase(vec_poses_time.begin());
      }
    }else{
        mclocalizer.updateData(vec_poses[0], vec_scan[0], vec_poses_time[0], vec_scan_time[0]);
      vec_scan.erase(vec_scan.begin());
      vec_scan_time.erase(vec_scan_time.begin());
      vec_poses.erase(vec_poses.begin());
      vec_poses_time.erase(vec_poses_time.begin());
        // vec_scan.pop_front();
        // vec_scan_time.pop_front();
        // vec_poses.pop_front();
        // vec_poses_time.pop_front();
    }
  }
  std::cout<<"CHECK-DATA end"<<std::endl;
}

void callback_scan(const sensor_msgs::PointCloud2::ConstPtr &msg){
    std::cout<<"CB - SCAN init"<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc_xyz);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_vx_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(pc_xyz);
    voxel_filter.setLeafSize(0.1, 0.1, 0.1);
    voxel_filter.filter(*pc_vx_filtered);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pc_vx_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.0, 2.0);  // 원하는 z 값 범위를 설정
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
        if(0.2 < dist && dist < 5) {
            usefulPoint++;
            eigenScan.conservativeResize(4, usefulPoint);
            eigenScan(0,usefulPoint-1) = currPoint.x;
            eigenScan(1,usefulPoint-1) = currPoint.y;
            eigenScan(2,usefulPoint-1) = currPoint.z;
            eigenScan(3,usefulPoint-1) = 1;
        }
    }
    vec_scan.emplace_back(eigenScan);
    // std::cout<<"CB - SCAN 5"<<std::endl;
    vec_scan_time.emplace_back(msg->header.stamp.toSec());
    check_data();
    std::cout<<"CB - SCAN END"<<std::endl;
}

void callback_pose(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::cout<<"CB-POSE init"<<std::endl;
    Eigen::Matrix4f eigenPose;
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    eigenPose<< m[0][0], m[0][1], m[0][2], msg->pose.pose.position.x,
                m[1][0], m[1][1], m[1][2], msg->pose.pose.position.y,
                m[2][0], m[2][1], m[2][2], msg->pose.pose.position.z,
                0,0,0,1;


    vec_poses.emplace_back(eigenPose);
    vec_poses_time.emplace_back(msg->header.stamp.toSec());
    check_data();
    std::cout<<"CB-POSE end"<<std::endl;
}