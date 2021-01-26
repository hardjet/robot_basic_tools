#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "camera_model/camera_models/Camera.h"

namespace camera_model {

class CameraFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraFactory();

  static boost::shared_ptr<CameraFactory> instance();

  static CameraPtr generateCamera(Camera::ModelType modelType, const std::string& cameraName,
                                  const cv::Size& imageSize);

  static CameraPtr generateCameraFromYamlFile(const std::string& filename);

 private:
  static boost::shared_ptr<CameraFactory> m_instance;
};

}  // namespace camera_model

#endif
