//
// Created by pyo on 24. 9. 19.
//
#ifndef MAP_RT_H
#define MAP_RT_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

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


class map_rt {
private:
    float mapWidth;
    float mapHeight;
    float mapResolution; // meter per pixel
    float mapCenterX;
    float mapCenterY;

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    Eigen::Matrix3f rotation;

    cv::Mat gridMap;
    Eigen::Matrix4f tf_pc2robot;

public:
    map_rt();
    ~map_rt();
    void updateMap(Eigen::Matrix4f pose, Eigen::Matrix4Xf scan);
    Eigen::Matrix3f get_rotation_matrix(float roll, float pitch, float yaw);
};

#endif //MAP_RT_H
