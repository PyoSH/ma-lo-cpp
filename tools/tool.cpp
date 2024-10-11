#include "tool.h"

namespace tool
{

cv::Mat cvMaptoMCLMap(cv::Mat mat)
{
  for(int i=0;i<mat.cols;i++)
  {
    for(int j=0;j<mat.rows;j++)
    {
      unsigned char val = mat.at<unsigned char>(j,i);
      if(val>100 && val<150) mat.at<unsigned char>(j,i)=255;
      // else if(val<128) mat.at<unsigned char>(j,i)=255-bool isInMap();val;
      else if(val>128) mat.at<unsigned char>(j,i)=255-val;
    }
  }
  return mat;
}


Eigen::Matrix4f mat2eigen(cv::Mat mat)
{
  Eigen::Matrix4f result =  Eigen::Matrix4f::Identity();

  result(0,0) = mat.at<float>(0,0);
  result(0,1) = mat.at<float>(0,1);
  result(0,2) = mat.at<float>(0,2);
  result(0,3) = mat.at<float>(0,3);

  result(1,0) = mat.at<float>(1,0);
  result(1,1) = mat.at<float>(1,1);
  result(1,2) = mat.at<float>(1,2);
  result(1,3) = mat.at<float>(1,3);

  result(2,0) = mat.at<float>(2,0);
  result(2,1) = mat.at<float>(2,1);
  result(2,2) = mat.at<float>(2,2);
  result(2,3) = mat.at<float>(2,3);

  result(3,0) = mat.at<float>(3,0);
  result(3,1) = mat.at<float>(3,1);
  result(3,2) = mat.at<float>(3,2);
  result(3,3) = mat.at<float>(3,3);

  return result;
}

cv::Mat eigen2mat(Eigen::Matrix4f mat)
{
  cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
  result.at<float>(0,0) = mat(0,0);
  result.at<float>(0,1) = mat(0,1);
  result.at<float>(0,2) = mat(0,2);
  result.at<float>(0,3) = mat(0,3);

  result.at<float>(1,0) = mat(1,0);
  result.at<float>(1,1) = mat(1,1);
  result.at<float>(1,2) = mat(1,2);
  result.at<float>(1,3) = mat(1,3);

  result.at<float>(2,0) = mat(2,0);
  result.at<float>(2,1) = mat(2,1);
  result.at<float>(2,2) = mat(2,2);
  result.at<float>(2,3) = mat(2,3);

  result.at<float>(3,0) = mat(3,0);
  result.at<float>(3,1) = mat(3,1);
  result.at<float>(3,2) = mat(3,2);
  result.at<float>(3,3) = mat(3,3);

  return result;
}

cv::Mat xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw)
{
  cv::Mat rot_vec = cv::Mat::zeros(3,1,CV_32FC1);
  rot_vec.at<float>(0) = roll;
  rot_vec.at<float>(1) = pitch;
  rot_vec.at<float>(2) = yaw;

  cv::Mat rot_mat;
  cv::Rodrigues(rot_vec,rot_mat);

  cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);

  rot_mat.copyTo(result(cv::Rect(0,0,3,3)));

  result.at<float>(0,3) = x;
  result.at<float>(1,3) = y;
  result.at<float>(2,3) = z;

  result.at<float>(3,3) = 1;

  return result;
}

void mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw)
{
  *x = mat.at<float>(0,3);
  *y = mat.at<float>(1,3);
  *z = mat.at<float>(2,3);

  cv::Mat rot_mat = cv::Mat(mat(cv::Rect(0,0,3,3)));

  cv::Mat rot_vec;
  cv::Rodrigues(rot_mat,rot_vec);
  *roll = rot_vec.at<float>(0);
  *pitch = rot_vec.at<float>(1);
  *yaw = rot_vec.at<float>(2);
}




Eigen::VectorXf eigen2xyzrpy(Eigen::Matrix4f mat)
{
  Eigen::VectorXf result(6);
  mat2xyzrpy(eigen2mat(mat), &result[0], &result[1], &result[2], &result[3], &result[4], &result[5]);
  return result;
}


Eigen::Matrix4f xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw)
{
  Eigen::Matrix4f result =  mat2eigen(xyzrpy2mat(x,y,z,roll,pitch,yaw));
  return result;
}

double GaussianRand()
{
  double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
  double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
  double r = u * u + v * v;
  if (r == 0 || r > 1) return GaussianRand();
  double c = sqrt(-2 * log(r) / r);
  return u * c;
}

Eigen::Matrix3f get_rotation_matrix(float roll, float pitch, float yaw) {
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

/*
* 지도 안에 있는지르 확인하기 위한 코드. 
* inside polygon test : Ray-casting 알고리즘
*/
bool isInside(const std::vector<cv::Point> points,double x, double y){
  bool isIn = false;
  for(int i=0; i<points.size(); ++i){
    int j = (i + points.size()-1) % points.size();
    if((points[i].y > y) != (points[j].y > y) 
      && (x < (points[j].x - points[i].x) * (y-points[i].y)/(points[j].y -points[i].y) + points[i].x)){
      isIn = !isIn;
    }   
  }
  std::cout << "isInside: " << x << " | " << y << " is " <<isIn << std::endl;
  return isIn;
}

/*
* 지도의 경계를 다각형으로 근사하기 -> ray-casting 알고리즘 써먹기 위해서!
*/
std::vector<cv::Point> getPolygonMap(const std::vector<cv::Point>& points, float epsilon){
  // 1. find 1st and last point
  cv::Point start = points.front();
  cv::Point end = points.back();

  // 2. find most far away point
  double dmax = 0;
  int idx = 0;
  for(int i=0; i<points.size(); ++i){
    double currDist = perpendicular_distance(points[i], start, end);
    if (currDist > dmax){
      idx = i;
      dmax = currDist;
    }
  }

  // 3. if the point is bigger than epsilon, divide with that point. 
  if(dmax > epsilon){
    std::vector<cv::Point> left(points.begin(), std::next(points.begin(), idx + 1));
    std::vector<cv::Point> right(std::next(points.begin(), idx), points.end());
    std::vector<cv::Point> result_left = getPolygonMap(left, epsilon);
    std::vector<cv::Point> result_right = getPolygonMap(right, epsilon);

    // 4. put them together(중복된 가장 먼 점을 제거하기 위해 result_left의 마지막 점을 제외)
    result_left.pop_back();
    result_left.insert(result_left.end(), result_right.begin(), result_right.end());
    return result_left;
  }else{
    return {start, end};
  }
}

/*
* 점과 직선(start, end) 사이의 직선 거리 반환
*/ 
double perpendicular_distance(const cv::Point point, const cv::Point start, const cv::Point end){ 
  if(start == end) return sqrt(pow(point.x-start.x,2) + pow(point.y-start.y,2));
  else{
    float line_length = sqrt(pow((end.y - start.y),2) + pow((end.x - start.x),2));
    return fabs((end.y - start.y) * point.x - (end.x - start.x) * point.y + end.x * start.y - end.y * start.x) / line_length;
  }  
} 

}