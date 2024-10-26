#include "map_RT.h"

map_rt::map_rt() {
    initMapWidth = 200;
    initMapHeight = 200;
    mapResolution = 0.05; //  meter per pixel
    mapCenterX= 0;
    mapCenterY= 0;
    mapCenterZ= 0;
    occuGridIncrease = -0.4; // default : 0.5 -> 0.02
    occuGridDecrease = 0.2; // default : 0.5

    gridMap = cv::Mat(initMapWidth, initMapHeight, CV_32FC1, cv::Scalar(0.5));

    currMapWidth = initMapWidth;
    currMapHeight = initMapHeight;
    offsetX = initMapWidth/2.0;
    offsetY = initMapHeight/2.0;

    roll_rad = -90 * M_PI / 180.0;
    pitch_rad = 0;
    yaw_rad = -90 * M_PI / 180.0;
    rotation = tool::get_rotation_matrix(roll_rad, pitch_rad, yaw_rad);
    tf_pc2robot.block<3,3>(0,0) << rotation;
    tf_pc2robot(0,3) = 0;
    tf_pc2robot(1,3) = 0;
    tf_pc2robot(2,3) = 0;
    tf_pc2robot(3,3) = 1;

}
map_rt::~map_rt() {
}

/*
 * 우선 2차원으로 진행. 그래도 3차원으로도 바로 될듯?
 */
void map_rt::updateMap(Eigen::Matrix4f pose, Eigen::Matrix4Xf scan, double t1, double t2) {
    
    if(is1stPose){
        mapCenterX = pose(0,3);
        mapCenterY = pose(1,3);
        mapCenterZ = pose(2,3);
        is1stPose = false;
    }

    expandMapIfNeeded(pose);

    int poseX_px, poseY_px;
    poseX_px = static_cast<int>((pose(0,3) - mapCenterX) / mapResolution + offsetX);
    poseY_px = static_cast<int>((pose(1,3) - mapCenterY) / mapResolution + offsetY);

    cv::Mat showMap;
    cv::cvtColor(gridMap, showMap, cv::COLOR_GRAY2RGB);
    cv::circle(showMap, cv::Point(poseX_px, poseY_px), 1, cv::Scalar(0, 0, 255), -1);
    Eigen::Matrix4Xf pcTransformed;

    scan = pose * tf_pc2robot * scan;
    pcTransformed = scan;
    // std::cout << "UPDATE - pc cnt:"<< pcTransformed.cols() << std::endl;
    for(int i=0; i<pcTransformed.cols(); i++) {
        int scanX_px;
        int scanY_px;
        scanX_px = static_cast<int>((pcTransformed(0,i) - mapCenterX) / mapResolution + offsetX);
        scanY_px = static_cast<int>((pcTransformed(1,i) - mapCenterY) / mapResolution + offsetY);

        if(0 <= scanX_px && scanX_px < gridMap.cols && 0 <= scanY_px && scanY_px < gridMap.rows) {
            cv::circle(showMap, cv::Point(scanX_px, scanY_px), 1, cv::Scalar(0, 255, 255), -1);
            cv::LineIterator it(gridMap, cv::Point(poseX_px, poseY_px), cv::Point(scanX_px, scanY_px), 8, cv::LINE_AA);
            for(int j=0; j<it.count-1; j++) {
                gridMap.at<float>(it.pos()) = gridMap.at<float>(it.pos()) + occuGridDecrease;
                it++;
            }

            gridMap.at<float>(cv::Point(scanX_px, scanY_px)) = gridMap.at<float>(cv::Point(scanX_px, scanY_px)) + occuGridIncrease;
        }
    }

    std::string text1 = "Current points: " + std::to_string(pcTransformed.cols());
    cv::putText(showMap, text1, cv::Point(10, currMapHeight - 100), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2);
    // cv::imshow("current Map", showMap);
    // cv::imshow("Map", gridMap);
    cv::Mat image_new = gridMap.clone();
    image_new.convertTo(image_new, CV_8UC3, 255.0);

    cv::Mat img_save = tool::cvMaptoMCLMap(image_new.clone(), 2);
    cv::imshow("dilate", img_save);
    
    cv::imwrite("map_.png", image_new);
    cv::imwrite("erodedMap_.png", img_save);

    double avg_time = (t1+t2)/2.0;
    // 1. PUB them into Image
    // sensor_msgs::ImagePtr map_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_new).toImageMsg();
    // map_msg->header.stamp = ros::Time(avg_time);
    // map_msg->header.frame_id = "map"; 
    // static ros::Publisher pub_map = ros::NodeHandle().advertise<sensor_msgs::Image>("/map", 1);
    // pub_map.publish(map_msg);
    
    // PUB2. pub them into Occupancy Grid Map topic
    convertAndPublishMap(image_new, avg_time);

    cv::waitKey(1);
}

void map_rt::convertAndPublishMap(const cv::Mat& image, const double t) {
    nav_msgs::OccupancyGrid map_msg;

    // 1. 메시지 헤더 설정
    map_msg.header.stamp = ros::Time(t);
    map_msg.header.frame_id = "odom";

    // 2. OccupancyGrid 메타정보 설정 (맵 크기, 해상도, 원점 설정)
    map_msg.info.resolution = mapResolution;   // 각 그리드 셀의 크기 (예: 0.1m)
    map_msg.info.width = image.cols;        // 맵의 너비 (그리드 셀 수)
    map_msg.info.height = image.rows;       // 맵의 높이 (그리드 셀 수)
    
    // 3. 원점 (맵의 좌표계에서의 원점)
    map_msg.info.origin.position.x = mapCenterX - currMapWidth*mapResolution/2;
    map_msg.info.origin.position.y = mapCenterY - currMapHeight*mapResolution/2;
    map_msg.info.origin.position.z = 0;
    map_msg.info.origin.orientation.w = 1.0;

    // 4. OccupancyGrid 데이터 채우기 (맵의 픽셀 값을 OccupancyGrid로 변환)
    map_msg.data.resize(map_msg.info.width * map_msg.info.height);

    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            int pixel_value = image.at<uchar>(y, x);  // 픽셀 값 가져오기 (0~255 범위)
            int index = y * image.cols + x;  // OccupancyGrid의 1D 인덱스

            // OpenCV 픽셀 값(0-255)을 OccupancyGrid 셀 값(-1, 0, 100)으로 변환
            if (pixel_value == 255) {
                map_msg.data[index] = 0;   // 255인 경우: 빈 공간
            } else if (pixel_value == 0) {
                map_msg.data[index] = 100; // 0인 경우: 장애물
            } else {
                map_msg.data[index] = -1;  // -1: 미확인 영역 (픽셀 값이 그 외인 경우)
            }
        }
    }

    // 5. ROS 토픽으로 발행
    static ros::Publisher pub_map = ros::NodeHandle().advertise<nav_msgs::OccupancyGrid>("/map", 1);
    pub_map.publish(map_msg);
}

void map_rt::expandMapIfNeeded(Eigen::Matrix4f pose) {
    // 로봇의 현재 위치 계산 (맵 좌표계 기준)
    int poseX_px = static_cast<int>((pose(0,3) - mapCenterX) / mapResolution + offsetX); // [px]
    int poseY_px = static_cast<int>((pose(1,3) - mapCenterY) / mapResolution + offsetY);// [px]
    bool condition1(false), condition2(false), condition3(false), condition4(false);

    cv::Rect roi(0,0, currMapWidth, currMapHeight);;
    bool expandNeeded = true;
    int margin = 80;
    // std::cout << "1" << std::endl;

    if (poseX_px > currMapWidth - margin) condition1 = true; // 오른쪽으로 확장
    else if (poseY_px > currMapHeight - margin) condition2 = true; // 아래로 확장
    else if (poseX_px < margin) condition3 = true; // 왼쪽으로 확장
    else if (poseY_px < margin) condition4 = true; // 위로 확장
    else expandNeeded = false;

    // std::cout << "2 " << std::endl;
    if(condition1){ // expand right side
        currMapWidth += initMapWidth;
        
    }else if(condition2){ // expand down side
        currMapHeight += initMapHeight;

    }else if (condition3){ // expand left side
        offsetX += initMapWidth;
        roi.x = initMapWidth;
        currMapWidth += initMapWidth;
        

    }else if (condition4){ // expand up side
        offsetY += initMapHeight;
        roi.y = initMapHeight;
        currMapHeight += initMapHeight;

    }
    // std::cout << "3 " << std::endl;

    if(expandNeeded){
        // std::cout << "expanded 1" << std::endl;
        cv::Mat newMap = cv::Mat::ones(currMapHeight, currMapWidth, gridMap.type()) * 0.5;
        std::cout << "expanded 2- currMapHeight " << currMapHeight <<"| currMapWidth "<< currMapWidth<< " " << roi<< std::endl;
        gridMap.copyTo(newMap(roi));
        // std::cout << "expanded 3" << std::endl;

        gridMap = newMap;

        std::cout << "Map expanded to new size: " << currMapWidth << "x" << currMapHeight << std::endl;
    }    
}
