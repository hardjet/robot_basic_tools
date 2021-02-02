#ifndef CAMERA_H
#define CAMERA_H

#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>

namespace camera_model {

class Ray {
 public:
  Ray();
  Ray(double theta, double phi);
  Ray(double x, double y, double z);
  explicit Ray(Eigen::Vector3d P);

  double& theta();
  double& phi();

  double theta() const;
  double phi() const;

 public:
  // (theta, phi) --> P
  Eigen::Vector3d toSpace() const;
  Eigen::Vector3d toSpace(double scale) const;

  void fromSpace(Eigen::Vector3d P);

  Ray& operator=(const Ray& other);

 protected:
  double m_theta;
  double m_phi;
};

class Camera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum ModelType { KANNALA_BRANDT = 0, MEI, PINHOLE, PINHOLE_FULL, SCARAMUZZA, POLYFISHEYE, FOV, SPLINE };

  class Parameters {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit Parameters(ModelType modelType);

    Parameters(ModelType modelType, const std::string& cameraName, int w, int h);

    ModelType& modelType();
    std::string& cameraName();
    int& imageWidth();
    int& imageHeight();

    ModelType modelType() const;
    const std::string& cameraName() const;
    int imageWidth() const;
    int imageHeight() const;
    cv::Size imageSize() const;

    int nIntrinsics() const;

    // virtual bool readFromYamlFile(const std::string& filename) = 0;
    // virtual void writeToYamlFile(const std::string& filename) const = 0;

   protected:
    ModelType m_modelType;
    int m_nIntrinsics;
    std::string m_cameraName;
    int m_imageWidth;
    int m_imageHeight;
  };

  virtual ModelType modelType() const = 0;
  virtual std::string& cameraName() = 0;
  virtual const std::string& cameraName() const = 0;
  virtual int imageWidth() const = 0;
  virtual int imageHeight() const = 0;
  virtual cv::Size imageSize() const = 0;
  virtual cv::Point2f getPrinciple() const = 0;

  virtual cv::Mat& mask();
  virtual const cv::Mat& mask() const;

  virtual void estimateIntrinsics(const cv::Size& boardSize, const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                  const std::vector<std::vector<cv::Point2f> >& imagePoints) = 0;

  virtual void setInitIntrinsics(const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                 const std::vector<std::vector<cv::Point2f> >& imagePoints) = 0;

  virtual void estimateExtrinsics(const std::vector<cv::Point3f>& objectPoints,
                                  const std::vector<cv::Point2f>& imagePoints, cv::Mat& rvec, cv::Mat& tvec) const;

  void estimateExtrinsics(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints,
                          Eigen::Matrix4d& Twc, cv::Mat& image) const;

  // Lift points from the image plane to the sphere
  virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
  //%output P

  // Lift points from the image plane to the projective space
  virtual void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
  //%output P

  // Lift points from the image plane to the projective space
  virtual void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P, float image_scale) const = 0;
  //%output P

  virtual void liftProjectiveToRay(const Eigen::Vector2d& p, Ray& ray) const = 0;

  virtual void rayToPlane(const Ray& ray, Eigen::Vector2d& p) const = 0;

  // Projects 3D points to the image plane (Pi function)
  virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const = 0;
  //%output p

  // Projects 3D points to the image plane (Pi function)
  virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p, float image_scalse) const = 0;
  //%output p

  // Projects 3D points to the image plane (Pi function)
  // and calculates jacobian
  virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p, Eigen::Matrix<double, 2, 3>& J) const = 0;
  //%output p
  //%output J

  virtual void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const = 0;
  //%output p

  // virtual void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0)
  // const = 0;
  virtual cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2, float fx = -1.0f, float fy = -1.0f,
                                          cv::Size imageSize = cv::Size(0, 0), float cx = -1.0f, float cy = -1.0f,
                                          cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const = 0;

  virtual int parameterCount() const = 0;

  virtual void readParameters(const std::vector<double>& parameters) = 0;
  virtual void writeParameters(std::vector<double>& parameters) const = 0;

  virtual void writeParametersToYamlFile(const std::string& filename) const = 0;

  virtual std::string parametersToString() const = 0;

  /**
   * \brief Calculates the reprojection distance between points
   *
   * \param P1 first 3D point coordinates
   * \param P2 second 3D point coordinates
   * \return euclidean distance in the plane
   */
  double reprojectionDist(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2) const;

  double reprojectionError(const std::vector<std::vector<cv::Point3f> >& objectPoints,
                           const std::vector<std::vector<cv::Point2f> >& imagePoints, const std::vector<cv::Mat>& rvecs,
                           const std::vector<cv::Mat>& tvecs, cv::OutputArray perViewErrors = cv::noArray()) const;
  double reprojectionRMSError(const std::vector<std::vector<cv::Point3f> >& objectPoints,
                              const std::vector<std::vector<cv::Point2f> >& imagePoints,
                              const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                              cv::OutputArray perViewErrors = cv::noArray()) const;

  double reprojectionError(const Eigen::Vector3d& P, const Eigen::Quaterniond& camera_q,
                           const Eigen::Vector3d& camera_t, const Eigen::Vector2d& observed_p) const;

  void projectPoints(const std::vector<cv::Point3f>& objectPoints, const cv::Mat& rvec, const cv::Mat& tvec,
                     std::vector<cv::Point2f>& imagePoints) const;

 protected:
  cv::Mat m_mask;
};

typedef boost::shared_ptr<Camera> CameraPtr;
typedef boost::shared_ptr<const Camera> CameraConstPtr;
}  // namespace camera_model

#endif
