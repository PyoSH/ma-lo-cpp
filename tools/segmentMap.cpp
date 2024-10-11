#include "tool.h"

int main(int argc,char **argv) {
    cv::Mat gridMap = cv::imread("/home/pyo/erodedMap.png", cv::IMREAD_GRAYSCALE);
    
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gridMap, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    double epsilon = 10.0;  // 허용 오차 값 (이미지 크기에 따라 조정 필요)
    std::vector<std::vector<cv::Point>> simplifiedContours;
    
    for (const auto& contour : contours) {
        std::vector<cv::Point> simplified = tool::getPolygonMap(contour, epsilon);
        simplifiedContours.push_back(simplified);
    }

    cv::Mat result = cv::Mat::zeros(gridMap.size(), CV_8UC3);
    for (const auto& simplified : simplifiedContours) {
        std::vector<std::vector<cv::Point>> drawContour = { simplified };
        cv::drawContours(result, drawContour, -1, cv::Scalar(0, 255, 0), 2); // 녹색으로 다각형 그리기
    }
    
    cv::imshow("Original Image", gridMap);
    cv::imshow("Polygon Approximation", result);
    cv::waitKey(0);

    return 0;
}