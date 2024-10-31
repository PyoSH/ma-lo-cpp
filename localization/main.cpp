#include "src/mcl.h"

std::deque<std::pair<double, Eigen::Matrix4f>> vec_poses;
std::deque<std::pair<double, Eigen::Matrix4Xf>> vec_scans; 
mcl mclocalizer;

// 데이터 콜백 함수 선언
void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg);
void callback_pose(const nav_msgs::Odometry::ConstPtr& msg);

// 타이머 콜백 함수 선언
void timerCallback(const ros::TimerEvent&);
void readWriteData();
void manageDataSize();

const size_t MAX_POSES_SIZE = 400;
const size_t MAX_SCANS_SIZE = 200;

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_mcl_test");
  std::cout << ros::this_node::getName() << std::endl;

  ros::NodeHandle nh;
  ros::Subscriber sub_scan = nh.subscribe<sensor_msgs::PointCloud2>("/merged_pointcloud", 100, callback_scan);
  ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry>("/b1_controller/odom", 100, callback_pose);

  // 타이머 설정 - 호출
  ros::Timer timer = nh.createTimer(ros::Duration(0.12), timerCallback);

  ros::spin();

  return 0;
}

void manageDataSize() {
  // vec_poses 크기 조절
  if (vec_poses.size() > MAX_POSES_SIZE) {
    vec_poses.erase(vec_poses.begin(), vec_poses.begin() + (vec_poses.size() - MAX_POSES_SIZE));
  }

  // vec_scans 크기 조절
  if (vec_scans.size() > MAX_SCANS_SIZE) {
    vec_scans.erase(vec_scans.begin(), vec_scans.begin() + (vec_scans.size() - MAX_SCANS_SIZE));
  }
}

void readWriteData(){
  // 0. writer에 대한 접근 제한
  mclocalizer.isPubOngoing = true; // 우선 이렇게. 나중에 ! 같은거 쓰자. 
  mclocalizer.canScanWrite = false;

  // 1. 가장 최근의 스캔 획득, 근사한 시간의 포즈 획득
  auto scan_it = vec_scans.back(); // 가장 최근의 스캔 데이터
  auto pose_it = std::lower_bound(vec_poses.begin(), vec_poses.end(), scan_it.first,
    [](const std::pair<double, Eigen::Matrix4f>& data, double time){
      return data.first < time;
    }); // 스캔 데이터와 근사한 시간의 포즈

  if(pose_it != vec_poses.end() && fabs(pose_it->first - scan_it.first) <= 0.1){
    // 2. predict with input pose
    mclocalizer.updatePredict(pose_it->second); // 스캔과 근사한 시간의 포즈 가져오기
    std::cout << "readWriteData 2-1 vec_poses: " << vec_poses.size() << std::endl;
    vec_poses.erase(vec_poses.begin(), pose_it + 1);

    // 3. update with input scan
    mclocalizer.updateScan(scan_it.second);
    vec_scans.clear();
    mclocalizer.canScanWrite = true;

    // 4. add motion: scan time with current time 
    auto latest_pose = vec_poses.back();
    // mclocalizer.updatePredict(latest_pose.second); // 가장 최근? 의 포즈 가져오기

    // 5. 발행하기
    mclocalizer.publishPose(latest_pose.second, latest_pose.first);
    // std::cout << "readWriteData 5-1 vec_poses: " << vec_poses.size() << std::endl;


  }else{
    std::cout << "readWriteData - dt is larger than 0.1 sec" << std::endl;
    manageDataSize();
  } 

  mclocalizer.isPubOngoing = false; 
  mclocalizer.canScanWrite = true;
}

void callback_scan(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  if(mclocalizer.canScanWrite){
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

    vec_scans.emplace_back(msg->header.stamp.toSec(), eigenScan);
  }else{
    std::cout << "CB-scan scan cannot writable" << std::endl;
  }
}

void callback_pose(const nav_msgs::Odometry::ConstPtr &msg) {
  Eigen::Matrix4f eigenPose;
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  eigenPose << m[0][0], m[0][1], m[0][2], msg->pose.pose.position.x,
                m[1][0], m[1][1], m[1][2], msg->pose.pose.position.y,
                m[2][0], m[2][1], m[2][2], msg->pose.pose.position.z,
                0, 0, 0, 1;

  // vec_poses.emplace_back(eigenPose);
  // vec_poses_time.emplace_back(msg->header.stamp.toSec());
  vec_poses.emplace_back(msg->header.stamp.toSec(), eigenPose);
  // mclocalizer.updatePredict(latest_pose);
  // mclocalizer.publishPose(latest_pose, latest_pose_time);
  // check_data();
}

void timerCallback(const ros::TimerEvent&){
  if(!mclocalizer.isPubOngoing && !vec_scans.empty() && !vec_poses.empty()){ // 발행되고있지 않으면
    readWriteData();
  }else{
    std::cout << "CB-timer : writing data is true" << std::endl;
  }
}