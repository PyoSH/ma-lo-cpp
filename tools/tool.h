#ifndef TOOL_H
#define TOOL_H
#include <vector>
#include <algorithm>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <eigen3/Eigen/Dense>

#include <pcl/point_types.h>  // 포인트 타입 정의
#include <pcl/point_cloud.h>  // 포인트 클라우드 클래스
#include <pcl_conversions/pcl_conversions.h>  // ros msg -> point cloud
#include <pcl/filters/voxel_grid.h>  // 다운샘플링 필터
#include <pcl/filters/passthrough.h>  // 범위 필터
#include <pcl/filters/statistical_outlier_removal.h>  // 이상치 제거 필터
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>  // 평면/모델 분할
#include <pcl/segmentation/extract_clusters.h>  // 클러스터링
#include <pcl/features/normal_3d.h>  // 법선 추정
#include <pcl/search/kdtree.h>  // KD 트리
#include <pcl/common/transforms.h>  // 변환 기능
#include <nanoflann.hpp>
namespace tool
{
  struct PointCloud {
    std::vector<cv::Point> points;

    // 포인트 개수 반환
    inline size_t kdtree_get_point_count() const { return points.size(); }

    // 인덱스로부터 x, y 좌표를 가져오기
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        return dim == 0 ? points[idx].x : points[idx].y;
    }

    // KD-트리에서 거리를 계산하는 함수
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
  };
  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 2>;

  cv::Mat cvMaptoMCLMap(cv::Mat mat);
  Eigen::Matrix4f mat2eigen(cv::Mat mat);
  cv::Mat eigen2mat(Eigen::Matrix4f mat);
  cv::Mat xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw);
  void mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw);
  Eigen::VectorXf eigen2xyzrpy(Eigen::Matrix4f mat);
  Eigen::Matrix4f xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw);
  double GaussianRand();
  Eigen::Matrix3f get_rotation_matrix(float roll, float pitch, float yaw);
  bool isInside(const std::vector<cv::Point> points,double x, double y, double minX, double minY, double maxX, double maxY);
  std::vector<cv::Point> getPolygonMap(const std::vector<cv::Point>& points, float epsilon);
  double perpendicular_distance(cv::Point point, cv::Point start, cv::Point end);
  void removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
  void removeGroundPlaneWithNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, float distance_threshold, float normal_threshold);
  cv::Point findNearestNeighborKDTree(const std::vector<cv::Point>& polygonPoints, const cv::Point& scanPoint);
}
#endif