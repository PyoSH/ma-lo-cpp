#include "map_RT.h"

map_rt::map_rt() {
    mapWidth = 300;
    mapHeight = 300;
    mapResolution = 0.05; // meter per pixel
    mapCenterX= 0;
    mapCenterY= 0;

    gridMap = cv::Mat(mapWidth, mapHeight, CV_32FC1, cv::Scalar(0.5));

    tf_pc2robot << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
}
map_rt::~map_rt() {

}

/*
 * 여기에서 점군 처리 등을 다 해야 한다.
 */
void map_rt::updateMap(Eigen::Matrix4f pose, Eigen::Matrix4f scan) {
    int poseX, poseY;
    poseX = static_cast<int>(pose(0,3) - mapCenterX + ((mapWidth * mapResolution)/2)/mapResolution);
    poseY = static_cast<int>(pose(1,3) - mapCenterY + ((mapWidth * mapResolution)/2)/mapResolution);

    cv::Mat showMap;
    cv::cvtColor(gridMap, showMap, cv::COLOR_GRAY2RGB);
    cv::circle(showMap, cv::Point(poseX, poseY), 1, cv::Scalar(0, 0, 255), -1);
    Eigen::Matrix4Xf pcTransformed;

    scan = pose * tf_pc2robot * scan;
    pcTransformed = scan;

    for(int i=0; i<pcTransformed.cols(); i++) {
        int scanX;
        int scanY;
        scanX = static_cast<int>(pcTransformed(0,i) - mapCenterX + ((mapWidth * mapResolution)/2)/mapResolution);
        scanY = static_cast<int>(pcTransformed(1,i) - mapCenterY + ((mapHeight * mapResolution)/2)/mapResolution);

        if(scanX >=0 && scanX < gridMap.cols && scanY >=0 && scanY < gridMap.rows) {
            cv::circle(showMap, cv::Point(scanX, scanY), 1, cv::Scalar(0, 0, 255), -1);
            cv::LineIterator it(gridMap, cv::Point(poseX, poseY), cv::Point(scanX, scanY), 8, cv::LINE_AA);

            for(int j=0; j<it.count-1; j++) {
                gridMap.at<float>(it.pos()) = gridMap.at<float>(it.pos()) + 0.02;
                it++;
            }

            gridMap.at<float>(cv::Point(scanX, scanY)) = gridMap.at<float>(cv::Point(scanX, scanY)) - 0.01;
        }
    }

    cv::imshow("current Map", showMap);
    cv::imshow("Map", gridMap);
    //  cv::Mat image_new = gridMap.clone();
    //  image_new.convertTo(image_new, CV_8UC3, 255.0);
    //  cv::imwrite("map.png", image_new);
    cv::waitKey(1);
}