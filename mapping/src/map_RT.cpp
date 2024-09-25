#include "map_RT.h"

map_rt::map_rt() {
    mapWidth = 800;
    mapHeight = 800;
    mapResolution = 0.1; //  meter per pixel
    mapCenterX= 0;
    mapCenterY= 0;

    gridMap = cv::Mat(mapWidth, mapHeight, CV_32FC1, cv::Scalar(0.5));

    // tf_pc2robot << 1, 0, 0, 0,
    //                 0, 1, 0, 0,
    //                 0, 0, 1, 0,
    //                 0, 0, 0, 1;
    // roll_rad = -90 * M_PI / 180.0;
    // pitch_rad = 0;
    // yaw_rad = -90 * M_PI / 180.0;
    roll_rad = -90 * M_PI / 180.0;
    pitch_rad = 0;
    yaw_rad = -90 * M_PI / 180.0;
    rotation = get_rotation_matrix(roll_rad, pitch_rad, yaw_rad);
    tf_pc2robot.block<3,3>(0,0) << rotation;
    tf_pc2robot(0,3) = 0;
    tf_pc2robot(1,3) = 0;
    tf_pc2robot(2,3) = 0;
    tf_pc2robot(3,3) = 1;

}
map_rt::~map_rt() {
}

/*
 * 여기에서 점군 처리 등을 다 해야 한다.
 */
void map_rt::updateMap(Eigen::Matrix4f pose, Eigen::Matrix4Xf scan) {
    int poseX, poseY;
    poseX = static_cast<int>((pose(0,3) - mapCenterX + ((mapWidth * mapResolution)/2))/mapResolution);
    poseY = static_cast<int>((pose(1,3) - mapCenterY + ((mapHeight * mapResolution)/2))/mapResolution);
    std::cout << "origin " << pose(0,3) << " "<< pose(1,3) <<" || " <<poseX << " " << poseY << std::endl;

    cv::Mat showMap;
    cv::cvtColor(gridMap, showMap, cv::COLOR_GRAY2RGB);
    cv::circle(showMap, cv::Point(poseX, poseY), 1, cv::Scalar(0, 0, 255), -1);
    Eigen::Matrix4Xf pcTransformed;

    scan = pose * tf_pc2robot * scan;
    pcTransformed = scan;
    std::cout << "UPDATE - pc cnt:"<< pcTransformed.cols() << std::endl;
    for(int i=0; i<pcTransformed.cols(); i++) {
        int scanX;
        int scanY;
        scanX = static_cast<int>((pcTransformed(0,i) - mapCenterX + ((mapWidth * mapResolution)/2))/mapResolution);
        scanY = static_cast<int>((pcTransformed(1,i) - mapCenterY + ((mapHeight * mapResolution)/2))/mapResolution);

        if(0 <= scanX && scanX < gridMap.cols && scanY >=0 && scanY < gridMap.rows) {
            cv::circle(showMap, cv::Point(scanX, scanY), 1, cv::Scalar(0, 255, 255), -1);
            cv::LineIterator it(gridMap, cv::Point(poseX, poseY), cv::Point(scanX, scanY), 8, cv::LINE_AA);
            // std::cout << "pc cnt:"<< pcTransformed.cols() << "|| it cnt:" << it.count << std::endl;
            for(int j=0; j<it.count-1; j++) {
                gridMap.at<float>(it.pos()) = gridMap.at<float>(it.pos()) + 0.02;
                it++;
            }

            gridMap.at<float>(cv::Point(scanX, scanY)) = gridMap.at<float>(cv::Point(scanX, scanY)) - 0.01;
        }
    }

    std::string text1 = "Current points: " + std::to_string(pcTransformed.cols());
    cv::putText(showMap, text1, cv::Point(10, mapHeight - 100), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2);
    cv::imshow("current Map", showMap);
    cv::imshow("Map", gridMap);
    cv::Mat image_new = gridMap.clone();
    image_new.convertTo(image_new, CV_8UC3, 255.0);
    cv::imwrite("map.png", image_new);
    cv::waitKey(1);
}

Eigen::Matrix3f map_rt::get_rotation_matrix(float roll, float pitch, float yaw) {
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