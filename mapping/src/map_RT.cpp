#include "map_RT.h"

map_rt::map_rt() {
    mapWidth = 900;
    mapHeight = 900;
    mapResolution = 0.1; //  meter per pixel
    mapCenterX= 0;
    mapCenterY= 0;
    mapCenterZ= 0;
    occuGridIncrease = -0.5; // default : 0.5 -> 0.02
    occuGridDecrease = 0.1; // default : 0.5

    gridMap = cv::Mat(mapWidth, mapHeight, CV_32FC1, cv::Scalar(0.5));

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

    int poseX_px, poseY_px;
    // poseX_px = static_cast<int>((pose(0,3) - mapCenterX + ((mapWidth * mapResolution)/2))/mapResolution);
    // poseY_px = static_cast<int>((pose(1,3) - mapCenterY + ((mapHeight * mapResolution)/2))/mapResolution);
    poseX_px = static_cast<int>((pose(0,3) - mapCenterX) / mapResolution + (mapWidth / 2));
    poseY_px = static_cast<int>((pose(1,3) - mapCenterY) / mapResolution + (mapHeight / 2));
    // poseZ = static_cast<int>((pose(2,3) - mapCenterZ + ((mapDepth * mapResolution)/2))/mapResolution);
    // std::cout << "origin " << pose(0,3) << " "<< pose(1,3) <<" || " <<poseX_px << " " << poseY_px << std::endl;

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
        // scanX_px = static_cast<int>((pcTransformed(0,i) - mapCenterX + ((mapWidth * mapResolution)/2))/mapResolution);
        // scanY_px = static_cast<int>((pcTransformed(1,i) - mapCenterY + ((mapHeight * mapResolution)/2))/mapResolution);
        scanX_px = static_cast<int>((pcTransformed(0,i) - mapCenterX) / mapResolution + (mapWidth / 2));
        scanY_px = static_cast<int>((pcTransformed(1,i) - mapCenterY) / mapResolution + (mapHeight / 2));

        if(0 <= scanX_px && scanX_px < gridMap.cols && 0 <= scanY_px && scanY_px < gridMap.rows) {
            cv::circle(showMap, cv::Point(scanX_px, scanY_px), 1, cv::Scalar(0, 255, 255), -1);
            cv::LineIterator it(gridMap, cv::Point(poseX_px, poseY_px), cv::Point(scanX_px, scanY_px), 8, cv::LINE_AA);
            // std::cout << "pc cnt:"<< pcTransformed.cols() << "|| it cnt:" << it.count << std::endl;
            for(int j=0; j<it.count-1; j++) {
                gridMap.at<float>(it.pos()) = gridMap.at<float>(it.pos()) + occuGridDecrease;
                it++;
            }

            gridMap.at<float>(cv::Point(scanX_px, scanY_px)) = gridMap.at<float>(cv::Point(scanX_px, scanY_px)) + occuGridIncrease;
            std::cout <<gridMap.at<float>(cv::Point(scanX_px, scanY_px)) << std::endl;
        }
    }

    std::string text1 = "Current points: " + std::to_string(pcTransformed.cols());
    cv::putText(showMap, text1, cv::Point(10, mapHeight - 100), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2);
    cv::imshow("current Map", showMap);
    cv::imshow("Map", gridMap);
    cv::Mat image_new = gridMap.clone();
    image_new.convertTo(image_new, CV_8UC3, 255.0);
    cv::imwrite("map.png", image_new);

    cv::Mat img_save = gridMap.clone();
    img_save.convertTo(img_save, CV_32FC1, 1.0 / 255.0);
    // cv::bitwise_not(img_save, img_save);    
    // for (int i = 0; i < img_save.rows; i++) {
    //     for (int j = 0; j < img_save.cols; j++) {
    //         // Get pixel value
    //         float pixel = img_save.at<uchar>(i, j);
    //         img_save.at<float>(i, j) = fabs(255 - img_save.at<float>(i, j));  // Black to White
    //     }
    // }
    cv::Mat inverted_image = 1.0 - img_save;  // Perform the inversion in floating point (CV_32FC1)

    // Optional: Convert back to CV_8UC1 for display or saving (if you need to visualize it)
    cv::Mat image_8bit;
    inverted_image.convertTo(image_8bit, CV_8UC1, 255.0);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(inverted_image, img_save, kernel);
    // img_save.convertTo(img_save, CV_8UC3, 0.0);
    cv::imshow("erode", img_save);
    // cv::imwrite("/home/pyo/erodedMap.png", img_save);

    double avg_time = (t1+t2)/2.0;
    // // 1. PUB them into Image
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
    map_msg.info.origin.position.x = mapCenterX - mapWidth*mapResolution/2;
    map_msg.info.origin.position.y = mapCenterY - mapHeight*mapResolution/2;
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