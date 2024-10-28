#ifndef MCL_H
#define MCL_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <opencv4/opencv2/opencv.hpp>
#include <tf/tf.h>
#include <random>
#include <cmath>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/point_types.h>  // 포인트 타입 정의
#include <pcl/point_cloud.h>  // 포인트 클라우드 클래스
#include <pcl_conversions/pcl_conversions.h>  // ros msg -> point cloud
#include <pcl/filters/voxel_grid.h>  // 다운샘플링 필터
#include <pcl/filters/passthrough.h>  // 범위 필터
#include <pcl/filters/statistical_outlier_removal.h>  // 이상치 제거 필터
#include <pcl/segmentation/sac_segmentation.h>  // 평면/모델 분할
#include <pcl/segmentation/extract_clusters.h>  // 클러스터링
#include <pcl/features/normal_3d.h>  // 법선 추정
#include <pcl/search/kdtree.h>  // KD 트리
#include <pcl/common/transforms.h>  // 변환 기능

#include "tool.h"

class mcl
{
    struct particle{
        Eigen::Matrix4f pose;
        double score;
        Eigen::Matrix4Xf scan; // Only for maximum probability particle.
    };

private:
    int m_sync_count;

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()

    float imageResolution;
    float mapCenterX;
    float mapCenterY;
    float mapCenterZ;
    float pxCenterX;
    float pxCenterY;

    
    float odomCovariance[6];
    int numOfParticle;
    std::vector<particle> particles;
    particle maxProbParticle;
    
    cv::Mat gridMap_show; // Gridmap for showing
    cv::Mat gridMap_use; // Gridmap for use (gaussian-blurred)
    cv::Mat likelihoodField;
    
    std::vector<cv::Point> mapCorners; // 1D array to check particle is inside contour of map
    int cornerXmin;
    int cornerYmin;
    int cornerXmax;
    int cornerYmax;

    double epsilon;
    Eigen::Matrix4f odomBefore;
    float minOdomDistance;
    float minOdomAngle;
    int repropagateCountNeeded;

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    Eigen::Matrix3f rotation;
    Eigen::Matrix4f tf_pc2robot;

    bool is1stPose;
    int predictionCounter;

    mcl::particle createRandomParticle();
    void initializeParticles();
    void prediction(Eigen::Matrix4f diffPose);
    void weightning(Eigen::Matrix4Xf scan);
    void resampling();
    void showInMap();
    cv::Mat createLikelihoodField(const cv::Mat& obstacleMap, double sigma_hit);
    double calculateScanLikelihood(const Eigen::Matrix4Xf scan, const Eigen::Matrix4f pose);

public:
  mcl();
  ~mcl();
  void updateData(Eigen::Matrix4f pose, Eigen::Matrix4Xf scan, double t1, double t2);
  void updatePredict(Eigen::Matrix4f pose);
};



#endif
