#include "src/mcl.h"
#include <chrono>
#include <mutex>

std::deque<Eigen::Matrix4f> vec_poses;
std::deque<double> vec_poses_time;
std::deque<Eigen::Matrix4Xf> vec_scan;
std::deque<double> vec_scan_time;

Eigen::Matrix4f latest_pose;
double latest_pose_time;
Eigen::Matrix4Xf latest_scan;
double latest_scan_time;

std::mutex mtx;

mcl mclocalizer;

// 데이터 콜백 함수 선언
void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg);
void callback_pose(const nav_msgs::Odometry::ConstPtr& msg);
void check_data();

// 타이머 콜백 함수 선언
void timerCallback(const ros::TimerEvent&);

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_mcl_test");
  std::cout << ros::this_node::getName() << std::endl;

  ros::NodeHandle nh;
  ros::Subscriber sub_scan = nh.subscribe<sensor_msgs::PointCloud2>("/merged_pointcloud", 100, callback_scan);
  ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry>("/b1_controller/odom", 100, callback_pose);

  // 타이머 설정 - 50ms마다 check_data 호출
  // ros::Timer timer = nh.createTimer(ros::Duration(0.04), timerCallback);

  ros::spin();

  return 0;
}

void check_data() {
  auto start_time = std::chrono::high_resolution_clock::now();

  while (!vec_poses.empty() && !vec_scan.empty()) {
      mclocalizer.updatePredict(vec_poses.front());
      mclocalizer.publishPose(vec_poses.front(), vec_poses_time.front());

      if (fabs(vec_poses_time.front() - vec_scan_time.front()) > 0.1) {
          if (vec_poses_time.front() > vec_scan_time.front()) {
              vec_scan.pop_front();
              vec_scan_time.pop_front();
          } else {
              vec_poses.pop_front();
              vec_poses_time.pop_front();
          }
      } else {
          mclocalizer.updateScan(vec_scan.front());
          vec_scan.pop_front();
          vec_scan_time.pop_front();
          vec_poses.pop_front();
          vec_poses_time.pop_front();
      }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

  std::cout << "check_data 실행 시간: " << duration << " microseconds" << std::endl;
}

void callback_scan(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *pc_xyz);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
  tool::removeGroundPlane(pc_xyz, pc_no_ground);

  int pointNum = pc_no_ground->points.size();
  Eigen::Matrix4Xf eigenScan = Eigen::Matrix4Xf::Ones(4, 1);
  int usefulPoint = 0;

  for (int i = 0; i < pointNum; i++) {
      pcl::PointXYZ currPoint = pc_no_ground->points[i];
      float dist = sqrt(pow(currPoint.x, 2) + pow(currPoint.y, 2) + pow(currPoint.z, 2));
      if (0.2 < dist && dist < 5) {
          usefulPoint++;
          eigenScan.conservativeResize(4, usefulPoint);
          eigenScan(0, usefulPoint - 1) = currPoint.x;
          eigenScan(1, usefulPoint - 1) = currPoint.y;
          eigenScan(2, usefulPoint - 1) = currPoint.z;
          eigenScan(3, usefulPoint - 1) = 1;
      }
  }

  // std::lock_guard<std::mutex> lock(mtx);
  // latest_scan = eigenScan;
  // latest_scan_time = msg->header.stamp.toSec();

  vec_scan.emplace_back(eigenScan);
  vec_scan_time.emplace_back(msg->header.stamp.toSec());
  check_data();
  // mclocalizer.updateScan(latest_scan);
  // mclocalizer.publishPose(latest_pose, latest_pose_time);
}

void callback_pose(const nav_msgs::Odometry::ConstPtr &msg) {
  Eigen::Matrix4f eigenPose;
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  eigenPose << m[0][0], m[0][1], m[0][2], msg->pose.pose.position.x,
                m[1][0], m[1][1], m[1][2], msg->pose.pose.position.y,
                m[2][0], m[2][1], m[2][2], msg->pose.pose.position.z,
                0, 0, 0, 1;

  vec_poses.emplace_back(eigenPose);
  vec_poses_time.emplace_back(msg->header.stamp.toSec());

  // std::lock_guard<std::mutex> lock(mtx);
  // latest_pose = eigenPose;
  // latest_pose_time = msg->header.stamp.toSec();

  // mclocalizer.updatePredict(latest_pose);
  // mclocalizer.publishPose(latest_pose, latest_pose_time);
  check_data();
}

// void timerCallback(const ros::TimerEvent&) {
//   check_data();
// }