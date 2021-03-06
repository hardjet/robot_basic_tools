#include "camera_model/camera_models/Camera.h"
#include "camera_model/camera_models/FovCamera.h"
#include "camera_model/camera_models/PolyFisheyeCamera.h"
#include "camera_model/camera_models/ScaramuzzaCamera.h"
#include "camera_model/camera_models/SplineCamera.h"
#include <camera_model/gpl/gpl.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace camera_model {

Camera::Parameters::Parameters(ModelType modelType) : m_modelType(modelType), m_imageWidth(0), m_imageHeight(0) {
  switch (modelType) {
    case KANNALA_BRANDT:
      m_nIntrinsics = 8;
      break;
    case PINHOLE:
      m_nIntrinsics = 8;
      break;
    case SCARAMUZZA:
      m_nIntrinsics = SCARAMUZZA_CAMERA_NUM_PARAMS;
      break;
    case POLYFISHEYE:
      m_nIntrinsics = FISHEYE_PARAMS_NUM;
      break;
    case SPLINE:
      m_nIntrinsics = SPLINE_PARAMS_NUM;
      break;
    case FOV:
      m_nIntrinsics = FOV_PARAM_NUM;
      break;
    case MEI:
    default:
      m_nIntrinsics = 9;
  }
}

Camera::Parameters::Parameters(ModelType modelType, const std::string& cameraName, int w, int h)
    : m_modelType(modelType), m_cameraName(cameraName), m_imageWidth(w), m_imageHeight(h) {
  switch (modelType) {
    case KANNALA_BRANDT:
      m_nIntrinsics = 8;
      break;
    case PINHOLE:
      m_nIntrinsics = 8;
      break;
    case SCARAMUZZA:
      m_nIntrinsics = SCARAMUZZA_CAMERA_NUM_PARAMS;
      break;
    case POLYFISHEYE:
      m_nIntrinsics = FISHEYE_PARAMS_NUM;
      break;
    case SPLINE:
      m_nIntrinsics = SPLINE_PARAMS_NUM;
      break;
    case FOV:
      m_nIntrinsics = FOV_PARAM_NUM;
      break;
    case MEI:
    default:
      m_nIntrinsics = 9;
  }
}

Camera::ModelType& Camera::Parameters::modelType() { return m_modelType; }

std::string& Camera::Parameters::cameraName() { return m_cameraName; }

int& Camera::Parameters::imageWidth() { return m_imageWidth; }

int& Camera::Parameters::imageHeight() { return m_imageHeight; }

Camera::ModelType Camera::Parameters::modelType() const { return m_modelType; }

const std::string& Camera::Parameters::cameraName() const { return m_cameraName; }

int Camera::Parameters::imageWidth() const { return m_imageWidth; }

int Camera::Parameters::imageHeight() const { return m_imageHeight; }

cv::Size Camera::Parameters::imageSize() const { return cv::Size(imageWidth(), imageHeight()); }

int Camera::Parameters::nIntrinsics() const { return m_nIntrinsics; }

cv::Mat& Camera::mask() { return m_mask; }

const cv::Mat& Camera::mask() const { return m_mask; }


void Camera::estimateExtrinsics(const std::vector<cv::Point3f>& objectPoints,
                                const std::vector<cv::Point2f>& imagePoints, cv::Mat& rvec, cv::Mat& tvec) const {
  unsigned int size_point = imagePoints.size();
  std::vector<cv::Point2f> Ms(size_point);
  size_t i;
  for (i = 0; i < size_point; i++) {
    Eigen::Vector3d P;
    liftProjective(Eigen::Vector2d(imagePoints[i].x, imagePoints[i].y), P);
    P /= P(2);
    Ms[i].x = float(P(0));
    Ms[i].y = float(P(1));
  }

  // assume unit focal length, zero principal point, and zero distortion
  // 得到世界坐标系到相机坐标系的变换矩阵
  cv::solvePnP(objectPoints, Ms, cv::Mat::eye(3, 3, CV_64F), cv::noArray(), rvec, tvec);
}

void Camera::estimateExtrinsics(const vector<cv::Point3f>& objectPoints, const vector<cv::Point2f>& imagePoints,
                                Eigen::Matrix4d& Twc, cv::Mat& image) const {
  unsigned int size_point = imagePoints.size();
  std::vector<cv::Point2f> Ms(size_point);
  size_t i;
  for (i = 0; i < size_point; i++) {
    Eigen::Vector3d P;
    liftProjective(Eigen::Vector2d(imagePoints[i].x, imagePoints[i].y), P);
    P /= P(2);
    Ms[i].x = float(P(0));
    Ms[i].y = float(P(1));
  }

  cv::Mat rvec;
  cv::Mat tvec;
  // assume unit focal length, zero principal point, and zero distortion
  // 得到标定板坐标系到相机坐标系的变换矩阵
  cv::solvePnP(objectPoints, Ms, cv::Mat::eye(3, 3, CV_64F), cv::noArray(), rvec, tvec);

  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  Eigen::Matrix3d Rcw;
  cv::cv2eigen(rotation, Rcw);
  Eigen::Vector3d tcw;
  cv::cv2eigen(tvec, tcw);
  // 计算相机坐标系到标定板坐标系的变换
  Twc.block<3, 3>(0, 0) = Rcw.transpose();
  Twc.block<3, 1>(0, 3) = -Rcw.transpose() * tcw;

  cv::putText(
      image,
      "t_wc: (m) " + std::to_string(Twc(0, 3)) + " " + std::to_string(Twc(1, 3)) + " " + std::to_string(Twc(2, 3)),
      cv::Point2f(50, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));

  std::vector<Eigen::Vector3d> axis;
  std::vector<cv::Point2f> img_pts;
  double size = 0.1;
  axis.emplace_back(objectPoints.at(0).x, objectPoints.at(0).y, objectPoints.at(0).z);
  axis.emplace_back(objectPoints.at(0).x + size, objectPoints.at(0).y, 0);
  axis.emplace_back(objectPoints.at(0).x, objectPoints.at(0).y + size, 0);
  axis.emplace_back(objectPoints.at(0).x, objectPoints.at(0).y, size);
  for (i = 0; i < axis.size(); ++i) {
    Eigen::Vector2d pt;
    Eigen::Vector3d Pt = Rcw * axis[i] + tcw;
    spaceToPlane(Pt, pt);  // 三维空间点，加上畸变投影到图像平面
    img_pts.emplace_back(pt.x(), pt.y());
  }

  cv::line(image, img_pts[0], img_pts[1], cv::Scalar(255, 0, 0), 4);
  cv::line(image, img_pts[0], img_pts[2], cv::Scalar(0, 255, 0), 4);
  cv::line(image, img_pts[0], img_pts[3], cv::Scalar(0, 0, 255), 4);
}

double Camera::reprojectionDist(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2) const {
  Eigen::Vector2d p1, p2;

  spaceToPlane(P1, p1);
  spaceToPlane(P2, p2);

  return (p1 - p2).norm();
}

double Camera::reprojectionError(const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                 const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                 const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                 cv::OutputArray _perViewErrors) const {
  int imageCount = objectPoints.size();
  size_t pointsSoFar = 0;
  double totalErr = 0.0;

  bool computePerViewErrors = _perViewErrors.needed();
  cv::Mat perViewErrors;
  if (computePerViewErrors) {
    _perViewErrors.create(imageCount, 1, CV_64F);
    perViewErrors = _perViewErrors.getMat();
  }

  int i = 0;
  // #pragma omp parallel for private(i)
  for (i = 0; i < imageCount; ++i) {
    size_t pointCount = imagePoints.at(i).size();

    pointsSoFar += pointCount;

    std::vector<cv::Point2f> estImagePoints;
    projectPoints(objectPoints.at(i), rvecs.at(i), tvecs.at(i), estImagePoints);

    double err = 0.0;
    for (size_t j = 0; j < imagePoints.at(i).size(); ++j) {
      err += cv::norm(imagePoints.at(i).at(j) - estImagePoints.at(j));
    }

    if (computePerViewErrors) {
      perViewErrors.at<double>(i) = err / pointCount;
    }

    totalErr += err;
  }

  return totalErr / pointsSoFar;
}

double Camera::reprojectionRMSError(const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                    const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                    const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                    cv::OutputArray _perViewErrors) const {
  int imageCount = objectPoints.size();
  size_t pointsSoFar = 0;
  double totalErr = 0.0;

  bool computePerViewErrors = _perViewErrors.needed();
  cv::Mat perViewErrors;
  if (computePerViewErrors) {
    _perViewErrors.create(imageCount, 1, CV_64F);
    perViewErrors = _perViewErrors.getMat();
  }

  for (int i = 0; i < imageCount; ++i) {
    size_t pointCount = imagePoints.at(i).size();

    pointsSoFar += pointCount;

    std::vector<cv::Point2f> estImagePoints;
    projectPoints(objectPoints.at(i), rvecs.at(i), tvecs.at(i), estImagePoints);

    double err = 0.0;
    for (size_t j = 0; j < imagePoints.at(i).size(); ++j) {
      err += cv::norm(imagePoints.at(i).at(j) - estImagePoints.at(j));

      //            std::cout<< imagePoints.at(i).at(j).x<<"
      //            "<<imagePoints.at(i).at(j).y
      //                     << " " << cv::norm(imagePoints.at(i).at(j) -
      //                     estImagePoints.at(j)) <<
      //                     std::endl;
    }

    if (computePerViewErrors) {
      perViewErrors.at<double>(i) = err / pointCount;
    }

    totalErr += err * err;
  }

  return sqrt(totalErr / pointsSoFar);
}

double Camera::reprojectionError(const Eigen::Vector3d& P, const Eigen::Quaterniond& camera_q,
                                 const Eigen::Vector3d& camera_t, const Eigen::Vector2d& observed_p) const {
  Eigen::Vector3d P_cam = camera_q.toRotationMatrix() * P + camera_t;

  Eigen::Vector2d p;
  spaceToPlane(P_cam, p);

  return (p - observed_p).norm();
}

void Camera::projectPoints(const std::vector<cv::Point3f>& objectPoints, const cv::Mat& rvec, const cv::Mat& tvec,
                           std::vector<cv::Point2f>& imagePoints) const {
  // project 3D object points to the image plane
  imagePoints.reserve(objectPoints.size());

  // double
  cv::Mat R0;
  cv::Rodrigues(rvec, R0);

  Eigen::MatrixXd R(3, 3);
  R << R0.at<double>(0, 0), R0.at<double>(0, 1), R0.at<double>(0, 2), R0.at<double>(1, 0), R0.at<double>(1, 1),
      R0.at<double>(1, 2), R0.at<double>(2, 0), R0.at<double>(2, 1), R0.at<double>(2, 2);

  Eigen::Vector3d t;
  t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

  for (size_t i = 0; i < objectPoints.size(); ++i) {
    const cv::Point3f& objectPoint = objectPoints.at(i);

    // Rotate and translate
    Eigen::Vector3d P;
    P << objectPoint.x, objectPoint.y, objectPoint.z;

    P = R * P + t;

    Eigen::Vector2d p;
    spaceToPlane(P, p);

    imagePoints.push_back(cv::Point2f(p(0), p(1)));
  }
}

Ray::Ray() : m_theta(0.0), m_phi(0.0) {}

Ray::Ray(double theta, double phi) : m_theta(theta), m_phi(phi) {}

Ray::Ray(double x, double y, double z) : m_theta(acos(z / sqrt(x * x + y * y + z * z))), m_phi(atan2(y, x)) {}

Ray::Ray(Eigen::Vector3d P) : m_theta(acos(P(2) / P.norm())), m_phi(atan2(P(1), P(0))) {}

double& Ray::theta() { return m_theta; }

double& Ray::phi() { return m_phi; }

double Ray::theta() const { return m_theta; }

double Ray::phi() const { return m_phi; }

Eigen::Vector3d Ray::toSpace() const {
  return Eigen::Vector3d(sin(m_theta) * cos(m_phi), sin(m_theta) * sin(m_phi), cos(m_theta));
}

Eigen::Vector3d Ray::toSpace(double scale) const {
  return Eigen::Vector3d(sin(m_theta) * cos(m_phi) * scale, sin(m_theta) * sin(m_phi) * scale, cos(m_theta) * scale);
}

void Ray::fromSpace(Eigen::Vector3d P) {
  m_theta = acos(P(2) / P.norm());
  m_phi = atan2(P(1), P(0));
}

Ray& Ray::operator=(const Ray& other) {
  if (this != &other) {
    m_theta = other.m_theta;
    m_phi = other.m_phi;
  }
  return *this;
}
}  // namespace camera_model
